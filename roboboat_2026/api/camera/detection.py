#!/usr/bin/env python3
"""
bbox_3d_estimator.py
--------------------
ROS2 node that fuses:
  - /oak/stereo/image_raw   (sensor_msgs/Image, 16UC1 depth in mm, aligned to RGB)
  - /oak/rgb/image_raw      (sensor_msgs/Image, used for frame_id / timing)
  - /oak/nn/spatial_detections (vision_msgs/Detection3DArray, bbox only used)

  + /oak/stereo/camera_info (sensor_msgs/CameraInfo)
  + /oak/rgb/camera_info    (sensor_msgs/CameraInfo)

Outputs:
  - /detections_3d          (vision_msgs/Detection3DArray, pose filled in)

Pipeline per detection
----------------------
1. Extract bbox center (u, v) in RGB pixel space from the 2D bounding box
   stored inside each Detection3D (uses BoundingBox2D if present, otherwise
   falls back to the bbox3d center projected via rgb camera_info).
2. Sample a small ROI around (u, v) in the aligned depth image.
3. Take the median of valid (non-zero, non-NaN) depth pixels in that ROI.
4. Back-project using stereo camera_info intrinsics:
       Z = depth_m
       X = (u - cx) * Z / fx
       Y = (v - cy) * Z / fy
5. Publish a Detection3DArray with pose.position filled and
   pose.orientation set to identity.

Coordinate frame: stereo/depth optical frame (right-hand, Z forward).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import message_filters
import numpy as np
import cv2

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, BoundingBox3D
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def intrinsics_from_camera_info(info: CameraInfo):
    """Return (fx, fy, cx, cy) from a CameraInfo message."""
    K = info.k  # row-major 3x3
    return K[0], K[4], K[2], K[5]  # fx, fy, cx, cy


def sample_depth_roi(depth_img: np.ndarray, u: float, v: float,
                     roi_px: int = 10) -> float:
    """
    Sample a square ROI of half-size roi_px around (u, v) in a depth image
    (uint16, millimetres).  Returns the median of valid pixels in metres,
    or NaN if no valid pixels exist.
    """
    h, w = depth_img.shape[:2]
    u, v = int(round(u)), int(round(v))

    u0, u1 = max(0, u - roi_px), min(w, u + roi_px + 1)
    v0, v1 = max(0, v - roi_px), min(h, v + roi_px + 1)

    roi = depth_img[v0:v1, u0:u1].astype(np.float32)
    valid = roi[(roi > 0) & np.isfinite(roi)]

    if valid.size == 0:
        return float('nan')

    return float(np.median(valid)) / 1000.0  # mm → m


def bbox_center_px(detection: Detection3D, rgb_fx: float, rgb_fy: float,
                   rgb_cx: float, rgb_cy: float):
    """
    Return (u, v) pixel center of the detection in the RGB image.

    OAK-D spatial detections store the normalised 2-D bounding box in
    detection.bbox (BoundingBox3D).  The x/y of bbox.center.position are
    the *normalised* [0..1] coordinates when coming from the MyriadX NN,
    but some pipelines write absolute pixel values — we handle both by
    checking whether the values are ≤ 1.0 (normalised) or > 1.0 (pixels).

    Fall-back: project the 3-D bbox centre with rgb intrinsics.
    """
    bbox: BoundingBox3D = detection.bbox
    cx3d = bbox.center.position.x
    cy3d = bbox.center.position.y

    # Heuristic: if both values are in (0, 1] treat as normalised
    # We need the image size for that — caller must pass it, or we rely on
    # camera_info width/height.  Here we return raw values and let the
    # caller decide (see node parameter `bbox_coords`).
    return cx3d, cy3d

def imgmsg_to_depth(msg: Image) -> np.ndarray:
    """Convert 16UC1 or 32FC1 Image msg to numpy without cv_bridge."""
    dtype = np.uint16 if msg.encoding == '16UC1' else np.float32
    return np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
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
        # 'normalised' | 'pixels'
        # OAK-D DepthAI typically gives normalised [0,1] bbox coords.
        self.declare_parameter('bbox_coords', 'pixels')
        self.declare_parameter('sync_slop_sec', 0.05)
        self.declare_parameter('output_frame', '')  # '' = use depth frame

        self._roi_px       = self.get_parameter('roi_half_px').value
        self._min_depth    = self.get_parameter('min_depth_m').value
        self._max_depth    = self.get_parameter('max_depth_m').value
        self._bbox_coords  = self.get_parameter('bbox_coords').value
        self._sync_slop    = self.get_parameter('sync_slop_sec').value
        self._output_frame = self.get_parameter('output_frame').value

        self._bridge = CvBridge()

        # ---- camera info (latched once) ------------------------------------
        self._rgb_info    = None
        self._stereo_info = None

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._sub_rgb_info = self.create_subscription(
            CameraInfo, '/oak/rgb/camera_info',
            self._cb_rgb_info, qos_profile=best_effort_qos)

        self._sub_stereo_info = self.create_subscription(
            CameraInfo, '/oak/stereo/camera_info',
            self._cb_stereo_info, qos_profile=best_effort_qos)

        # ---- synchronised subscribers --------------------------------------
        self._sub_depth = message_filters.Subscriber(
            self, Image, '/oak/stereo/image_raw',
            qos_profile=best_effort_qos)

        self._sub_rgb = message_filters.Subscriber(
            self, Image, '/oak/rgb/image_raw',
            qos_profile=best_effort_qos)

        self._sub_dets = message_filters.Subscriber(
            self, Detection3DArray, '/oak/nn/spatial_detections',
            qos_profile=best_effort_qos)

        slop = self._sync_slop
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_depth, self._sub_rgb, self._sub_dets],
            queue_size=10,
            slop=slop,
        )
        self._sync.registerCallback(self._cb_sync)

        # ---- publisher -----------------------------------------------------
        self._pub = self.create_publisher(
            Detection3DArray, '/detections_3d', 10)

        self.get_logger().info('bbox_3d_estimator ready — waiting for camera_info …')

    # -----------------------------------------------------------------------
    # Camera info callbacks (latch once)
    # -----------------------------------------------------------------------

    def _cb_rgb_info(self, msg: CameraInfo):
        if self._rgb_info is None:
            self._rgb_info = msg
            self.get_logger().info(
                f'RGB camera_info received: {msg.width}x{msg.height}')

    def _cb_stereo_info(self, msg: CameraInfo):
        if self._stereo_info is None:
            self._stereo_info = msg
            self.get_logger().info(
                f'Stereo camera_info received: {msg.width}x{msg.height}')

    # -----------------------------------------------------------------------
    # Main synchronised callback
    # -----------------------------------------------------------------------

    def _cb_sync(self, depth_msg: Image, rgb_msg: Image,
                 dets_msg: Detection3DArray):

        if self._rgb_info is None or self._stereo_info is None:
            self.get_logger().warn(
                'camera_info not yet received — skipping frame',
                throttle_duration_sec=2.0)
            return

        if not dets_msg.detections:
            return

        # ---- decode depth image -------------------------------------------
        try:
            # 16UC1: uint16 millimetres
            depth_cv = imgmsg_to_depth(depth_msg)
        except Exception as e:
            self.get_logger().error(f'cv_bridge depth error: {e}')
            return

        if depth_cv.dtype != np.uint16:
            # Some pipelines publish 32FC1 (metres already)
            if depth_cv.dtype == np.float32:
                depth_cv = (depth_cv * 1000).astype(np.uint16)
            else:
                self.get_logger().error(
                    f'Unexpected depth dtype: {depth_cv.dtype}')
                return

        depth_h, depth_w = depth_cv.shape[:2]

        # ---- intrinsics ---------------------------------------------------
        # Stereo (depth) intrinsics for back-projection
        sfx, sfy, scx, scy = intrinsics_from_camera_info(self._stereo_info)
        # RGB intrinsics for normalised→pixel conversion
        rfx, rfy, rcx, rcy = intrinsics_from_camera_info(self._rgb_info)
        rgb_w = self._rgb_info.width
        rgb_h = self._rgb_info.height

        # ---- build output message -----------------------------------------
        out_msg = Detection3DArray()
        out_msg.header.stamp    = dets_msg.header.stamp
        out_msg.header.frame_id = (self._output_frame
                                   or self._stereo_info.header.frame_id
                                   or depth_msg.header.frame_id)

        for det in dets_msg.detections:
            raw_u = det.bbox.center.position.x
            raw_v = det.bbox.center.position.y

            # Convert bbox centre to depth-image pixel coords
            if self._bbox_coords == 'normalised':
                # OAK-D NN outputs are normalised [0, 1]
                u_rgb = raw_u * rgb_w
                v_rgb = raw_v * rgb_h
            else:
                u_rgb = raw_u
                v_rgb = raw_v

            # Scale from RGB resolution to depth resolution
            # (they are aligned but may differ in resolution)
            scale_x = depth_w / rgb_w
            scale_y = depth_h / rgb_h
            u_d = u_rgb * scale_x
            v_d = v_rgb * scale_y

            # ---- sample depth ---------------------------------------------
            depth_m = sample_depth_roi(depth_cv, u_d, v_d, self._roi_px)

            if np.isnan(depth_m):
                self.get_logger().debug(
                    f'No valid depth for detection at ({u_d:.1f},{v_d:.1f})')
                continue

            if not (self._min_depth <= depth_m <= self._max_depth):
                self.get_logger().debug(
                    f'Depth {depth_m:.3f}m out of range — skipping')
                continue

            # ---- back-project to 3-D (stereo optical frame) ---------------
            # Use stereo intrinsics (depth is in stereo frame)
            # u_d, v_d are in depth pixel space; re-map to stereo principal pt
            X = (u_d - scx) * depth_m / sfx
            Y = (v_d - scy) * depth_m / sfy
            Z = depth_m

            # ---- build Detection3D ----------------------------------------
            out_det = Detection3D()
            out_det.header = out_msg.header

            # Copy results, bbox size, and class info from input
            out_det.results    = det.results
            out_det.bbox       = det.bbox

            # Fill in the estimated 3-D pose
            out_det.bbox.center.position.x = X
            out_det.bbox.center.position.y = Y
            out_det.bbox.center.position.z = Z
            # Identity orientation (no rotation estimated)
            out_det.bbox.center.orientation = Quaternion(
                x=0.0, y=0.0, z=0.0, w=1.0)

            out_msg.detections.append(out_det)

            self.get_logger().debug(
                f'Detection → X={X:.3f} Y={Y:.3f} Z={Z:.3f} m '
                f'(u={u_d:.1f} v={v_d:.1f})')

        if out_msg.detections:
            self._pub.publish(out_msg)
            self.get_logger().debug(
                f'Published {len(out_msg.detections)} detection(s)')


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