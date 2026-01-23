#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import threading


class RGBDepthRecorder(Node):
    def __init__(self):
        super().__init__('rgb_depth_recorder')

        self.bridge = CvBridge()

        # Topics
        self.rgb_topic = '/oak/rgb/image_raw'
        self.depth_topic = '/oak/stereo/image_raw'

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, self.rgb_topic, self.rgb_callback, 10
        )

        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, 10
        )

        # Video writers (lazy init)
        self.rgb_writer = None
        self.depth_writer = None

        self.fps = 30
        self.codec = cv2.VideoWriter_fourcc(*'mp4v')

        self.lock = threading.Lock()

        self.get_logger().info("RGB & Depth recorder started")

    # ===================== RGB =====================
    def rgb_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB conversion failed: {e}")
            return

        h, w, _ = frame.shape

        with self.lock:
            if self.rgb_writer is None:
                self.rgb_writer = cv2.VideoWriter(
                    'rgb.mp4', self.codec, self.fps, (w, h)
                )
                self.get_logger().info("RGB video writer initialized")

            self.rgb_writer.write(frame)

    # ===================== DEPTH =====================
    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")
            return

        # Handle depth format
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0  # mm → meters
        else:
            depth_m = depth.astype(np.float32)

        # Clip range for visualization (adjust as needed)
        depth_m = np.nan_to_num(depth_m)
        depth_m = np.clip(depth_m, 0.2, 10.0)

        # Normalize to 0–255
        depth_norm = cv2.normalize(
            depth_m, None, 0, 255, cv2.NORM_MINMAX
        ).astype(np.uint8)

        # Apply colormap
        depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)

        h, w, _ = depth_color.shape

        with self.lock:
            if self.depth_writer is None:
                self.depth_writer = cv2.VideoWriter(
                    'depth.mp4', self.codec, self.fps, (w, h)
                )
                self.get_logger().info("Depth video writer initialized")

            self.depth_writer.write(depth_color)

    # ===================== CLEANUP =====================
    def destroy_node(self):
        self.get_logger().info("Shutting down, releasing video writers")

        with self.lock:
            if self.rgb_writer:
                self.rgb_writer.release()
            if self.depth_writer:
                self.depth_writer.release()

        super().destroy_node()


def main():
    rclpy.init()
    node = RGBDepthRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
