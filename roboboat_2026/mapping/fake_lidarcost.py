#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import OccupancyGrid


class TestCostmapPublisher(Node):
    def __init__(self):
        super().__init__('test_costmap_publisher')

        self.pub = self.create_publisher(
            OccupancyGrid,
            '/test_costmap',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_costmap)

        # Map parameters
        self.resolution = 0.1  # meters per cell
        self.width = 100       # 10m / 0.1
        self.height = 100

        # Robot-centered origin
        self.origin_x = - (self.width * self.resolution) / 2.0
        self.origin_y = - (self.height * self.resolution) / 2.0

        # Robot cell index
        self.robot_x = self.width // 2
        self.robot_y = self.height // 2

        self.get_logger().info("Test costmap publisher started")

    def publish_costmap(self):
        # Initialize grid (free space)
        grid = np.zeros((self.height, self.width), dtype=np.int8)

        # --------------------------------------------------
        # Create obstacle on front-left of robot
        # --------------------------------------------------
        front_offset = 10   # cells (1.0 m)
        left_offset = 5     # cells (0.5 m)

        obs_x = self.robot_x + front_offset
        obs_y = self.robot_y + left_offset

        # Make obstacle a small block (3x3)
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                y = obs_y + dy
                x = obs_x + dx
                if 0 <= x < self.width and 0 <= y < self.height:
                    grid[y, x] = 100  # occupied

        # --------------------------------------------------
        # Build OccupancyGrid message
        # --------------------------------------------------
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = grid.flatten().tolist()

        self.pub.publish(msg)
        self.get_logger().info("Published test occupancy grid")


def main(args=None):
    rclpy.init(args=args)
    node = TestCostmapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
