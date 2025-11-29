#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Image
from threading import Thread, Lock
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
class FTPNode(Node):
    def __init__(self):
        super().__init__('FTPNode')
        
        self.logger = self.get_logger()
        self.logger.info("Follow the path Node Started")
        self.bridge = CvBridge()

        # Proportional control gains
        self.k_sway = 0.001
        self.k_yaw = 0.001
        self.k_surge = 0.3

        # camera frame attributes
        self.frame_width = None
        self.frame_height = None

        # Subscribe to normalized surge/sway/yaw command
        self.sub = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.listener_callback,
            10
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            '/teensy/pwm',
            10
        )

    # -----------------------------
    # Buoy Detection Functions
    # -----------------------------
    def detect_red_buoys(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
        return self.get_contour_centers(mask_red)

    def detect_green_buoys(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([90, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        return self.get_contour_centers(mask_green)

    def get_contour_centers(self, mask, min_area=100):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centers = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            centers.append((cx, cy))
        return centers

    # -----------------------------
    # Midpoint Navigation
    # -----------------------------
    def compute_midpoint_target(self, red_buoys, green_buoys):
        """
        Compute the midpoint between the closest red and green buoys.
        Returns (target_x, target_y)
        """
        if red_buoys and green_buoys:
            # Find closest red and green buoy pair (largest y)
            red_closest = max(red_buoys, key=lambda b: b[1])
            green_closest = max(green_buoys, key=lambda b: b[1])

            self.logger.info(f"red: {red_closest}")
            self.logger.info(f"green: {green_closest}")

            # Midpoint
            target_x = (red_closest[0] + green_closest[0]) // 2
            target_y = (red_closest[1] + green_closest[1]) // 2
            return target_x, target_y
        elif red_buoys:
            red_closest = max(red_buoys, key=lambda b: b[1])
            self.logger.info(f"red: {red_closest}")
            target_x = (red_closest[0]) // 2
            target_y = (red_closest[1]) // 2
            return target_x + 100, target_y

        elif green_buoys:
            green_closest = max(green_buoys, key=lambda b: b[1])
            self.logger.info(f"green: {green_closest}")
            target_x = (green_closest[0]) // 2
            target_y = (green_closest[1]) // 2
            return target_x - 100, target_y
        else:
            return self.frame_width//2, self.frame_height//2

    # -----------------------------
    # Main Navigation Function
    # -----------------------------
    def FTPlogic(self, frame):
        # Detect buoys
        red_buoys = self.detect_red_buoys(frame)
        green_buoys = self.detect_green_buoys(frame)

        # Compute target midpoint
        target_x, target_y = self.compute_midpoint_target(red_buoys, green_buoys)

        # Initialize control
        surge = self.k_surge
        sway = 0.0
        yaw = 0.0

        if target_x is not None:
            error_x = target_x - self.frame_width / 2
            sway = self.k_sway * error_x
            yaw = -self.k_yaw * error_x

        # Clip outputs
        surge = np.clip(surge, -0.6, 0.6)
        sway = np.clip(sway, -0.3, 0.3)
        yaw = np.clip(yaw, -0.2, 0.2)

        return [surge, sway, yaw]
    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.frame_width is None or self.frame_height is None:
            self.frame_height, self.frame_width = cv_image.shape[:2]
            self.logger.info(f"Width: f{self.frame_width}")
            self.logger.info(f"Height: f{self.frame_height}")

        cmd = self.FTPlogic(cv_image)

        msg = Float32MultiArray()
        msg.data = cmd

        self.pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Stopping FTP mission...")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FTPNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C received")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
