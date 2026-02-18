#!/usr/bin/env python3
"""
bbox_3d_estimator.py
--------------------
ROS2 node that fuses:
  - /oak/stereo/image_raw      (sensor_msgs/Image, 16UC1 depth in mm, aligned to RGB)
  - /oak/nn/spatial_detections (vision_msgs/Detection3DArray, bbox only used)

  + /oak/stereo/camera_info    (sensor_msgs/CameraInfo)
  + /oak/rgb/camera_info       (sensor_msgs/CameraInfo)

Outputs:
  - /detections_3d             (vision_msgs/Detection3DArray, pose filled in)

Pipeline
--------
- Depth image and camera infos are cached on arrival.
- On each detection message, the latest cached depth is used immediately —
  no synchronisation needed.
- For each bbox:
    1. Extract center (u, v) in RGB pixel space.
    2. Scale (u, v) to depth image resolution.
    3. Sample median depth over a small ROI around (u, v).
    4. Back-project to 3-D:
         Z = depth_m
         X = (u - cx) * Z / fx
         Y = (v - cy) * Z / fy
    5. Publish Detection3DArray with pose filled in.

Coordinate frame: stereo/depth optical frame (Z forward, right-hand).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D
from geometry_msgs.msg import Quaternion


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def intrinsics_from_camera_info(info: CameraInfo):
    """Return (fx, fy, cx, cy) from a CameraInfo K matrix."""
    K = info.k  # row-major 3x3
    return K[0], K[4], K[2], K[5]


def imgmsg_to_depth(msg: Image) -> np.ndarray:
    """Convert 16UC1 or 32FC1 Image msg to a numpy array without cv_bridge."""
    if msg.encoding == '16UC1':
        dtype = np.uint16
    elif msg.encoding == '32FC1':
        dtype = np.float32
    else:
        raise ValueError(f'Unsupported depth encoding: {msg.encoding}')
    return np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)


def sample_depth_roi(depth_img: np.ndarray, u: float, v: float,
                     roi_px: int = 10) -> float:
    """
    Return the median valid depth (metres) in a square ROI of half-size
    roi_px centred on (u, v).  Returns NaN if no valid pixels exist.

    Expects uint16 (millimetres) or float32 (metres already).
    """
    h, w = depth_img.shape[:2]
    ui, vi = int(round(u)), int(round(v))

    u0, u1 = max(0, ui - roi_px), min(w, ui + roi_px + 1)
    v0, v1 = max(0, vi - roi_px), min(h, vi + roi_px + 1)

    roi = depth_img[v0:v1, u0:u1].astype(np.float32)
    valid = roi[(roi > 0) & np.isfinite(roi)]

    if valid.size == 0:
        return float('nan')

    median = float(np.median(valid))

    # Convert mm -> m for uint16 source
    if depth_img.dtype == np.uint16:
        median /= 1000.0

    return median


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class BBox3DEstimator(Node):

    def __init__(self):
        super().__init__('bbox_3d_estimator')

        # ---- parameters ----------------------------------------------------
        self.declare_parameter('roi_half_px', 10)
        self.declare_parameter('min_depth_m', 0.1)
        self.declare_parameter('max_depth_m', 15.0)
        self.declare_parameter('bbox_coords', 'pixels')   # 'pixels' | 'normalised'
        self.declare_parameter('output_frame', '')         # '' = use depth frame

        self._roi_px       = self.get_parameter('roi_half_px').value
        self._min_depth    = self.get_parameter('min_depth_m').value
        self._max_depth    = self.get_parameter('max_depth_m').value
        self._bbox_coords  = self.get_parameter('bbox_coords').value
        self._output_frame = self.get_parameter('output_frame').value

        # ---- state ---------------------------------------------------------
        self._latest_depth: Image | None = None
        self._rgb_info:     CameraInfo | None = None
        self._stereo_info:  CameraInfo | None = None

        # ---- QoS -----------------------------------------------------------
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- subscribers ---------------------------------------------------
        self.create_subscription(
            CameraInfo, '/oak/rgb/camera_info',
            self._cb_rgb_info, qos_profile=be_qos)

        self.create_subscription(
            CameraInfo, '/oak/stereo/camera_info',
            self._cb_stereo_info, qos_profile=be_qos)

        # Cache latest depth — no processing here
        self.create_subscription(
            Image, '/oak/stereo/image_raw',
            self._cb_depth, qos_profile=be_qos)

        # All work happens here
        self.create_subscription(
            Detection3DArray, '/oak/nn/spatial_detections',
            self._cb_detections, qos_profile=be_qos)

        # ---- publisher -----------------------------------------------------
        self._pub = self.create_publisher(
            Detection3DArray, '/detections_3d', 10)

        self.get_logger().info('bbox_3d_estimator ready.')

    # -----------------------------------------------------------------------
    # Cache callbacks
    # -----------------------------------------------------------------------

    def _cb_rgb_info(self, msg: CameraInfo):
        if self._rgb_info is None:
            self._rgb_info = msg
            self.get_logger().info(
                f'RGB camera_info latched: {msg.width}x{msg.height}')

    def _cb_stereo_info(self, msg: CameraInfo):
        if self._stereo_info is None:
            self._stereo_info = msg
            self.get_logger().info(
                f'Stereo camera_info latched: {msg.width}x{msg.height}')

    def _cb_depth(self, msg: Image):
        self._latest_depth = msg

    # -----------------------------------------------------------------------
    # Detection callback — all the work happens here
    # -----------------------------------------------------------------------

    def _cb_detections(self, dets_msg: Detection3DArray):

        # ---- guards --------------------------------------------------------
        if self._latest_depth is None:
            self.get_logger().warn('No depth image yet — skipping.',
                                   throttle_duration_sec=2.0)
            return

        if self._rgb_info is None or self._stereo_info is None:
            self.get_logger().warn('camera_info not yet received — skipping.',
                                   throttle_duration_sec=2.0)
            return

        if not dets_msg.detections:
            return

        # ---- decode depth --------------------------------------------------
        try:
            depth_cv = imgmsg_to_depth(self._latest_depth)
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        depth_h, depth_w = depth_cv.shape[:2]

        # ---- intrinsics ----------------------------------------------------
        sfx, sfy, scx, scy = intrinsics_from_camera_info(self._stereo_info)
        rgb_w = self._rgb_info.width
        rgb_h = self._rgb_info.height

        # Scaling factors: RGB pixel space -> depth image space
        scale_x = depth_w / rgb_w
        scale_y = depth_h / rgb_h

        # ---- build output message ------------------------------------------
        out_msg = Detection3DArray()
        out_msg.header.stamp    = dets_msg.header.stamp
        out_msg.header.frame_id = (self._output_frame
                                   or self._stereo_info.header.frame_id
                                   or self._latest_depth.header.frame_id)

        # ---- process each detection ----------------------------------------
        for det in dets_msg.detections:

            raw_u = det.bbox.center.position.x
            raw_v = det.bbox.center.position.y

            # Convert to RGB pixel space if needed
            if self._bbox_coords == 'normalised':
                u_rgb = raw_u * rgb_w
                v_rgb = raw_v * rgb_h
            else:
                u_rgb = raw_u
                v_rgb = raw_v

            # Scale to depth image space
            u_d = u_rgb * scale_x
            v_d = v_rgb * scale_y

            # Sample depth in ROI around projected center
            depth_m = sample_depth_roi(depth_cv, u_d, v_d, self._roi_px)

            if np.isnan(depth_m):
                self.get_logger().warn(
                    f'No valid depth at ({u_d:.1f}, {v_d:.1f}) — skipping.',
                    throttle_duration_sec=1.0)
                continue

            if not (self._min_depth <= depth_m <= self._max_depth):
                self.get_logger().debug(
                    f'Depth {depth_m:.3f} m out of range — skipping.')
                continue

            # Back-project (u, v, Z) -> (X, Y, Z) in stereo optical frame
            X = (u_d - scx) * depth_m / sfx
            Y = (v_d - scy) * depth_m / sfy
            Z = depth_m

            # Build output detection (copy class + bbox size from input)
            out_det = Detection3D()
            out_det.header  = out_msg.header
            out_det.results = det.results
            out_det.bbox    = det.bbox

            out_det.bbox.center.position.x  = X
            out_det.bbox.center.position.y  = Y
            out_det.bbox.center.position.z  = Z
            out_det.bbox.center.orientation = Quaternion(
                x=0.0, y=0.0, z=0.0, w=1.0)

            out_msg.detections.append(out_det)

            self.get_logger().info(
                f'Detection -> X={X:.3f}  Y={Y:.3f}  Z={Z:.3f} m  '
                f'(depth px: {u_d:.1f}, {v_d:.1f})')

        if out_msg.detections:
            self._pub.publish(out_msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = BBox3DEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()