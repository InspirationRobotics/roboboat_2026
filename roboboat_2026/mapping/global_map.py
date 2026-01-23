#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class EmptyGlobalMapNode(Node):
    def __init__(self):
        super().__init__('empty_global_map_node')
        
        # Declare parameters
        self.declare_parameter('map_resolution', 1.0)  # meters per cell
        self.declare_parameter('map_width', 500)       # cells (500m x 500m area)
        self.declare_parameter('map_height', 500)      # cells
        self.declare_parameter('publish_rate', 1.0)    # Hz
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('rolling_window', True)  # Center on robot
        
        # Get parameters
        self.resolution = self.get_parameter('map_resolution').value
        self.width = self.get_parameter('map_width').value
        self.height = self.get_parameter('map_height').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rolling_window = self.get_parameter('rolling_window').value
        
        # Publisher
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            10
        )
        
        # Timer for periodic publishing
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_map
        )
        
        # Create empty map data (all unknown or all free space)
        # -1 = unknown, 0 = free, 100 = occupied
        self.map_data = np.zeros(self.width * self.height, dtype=np.int8)
        # Use 0 for free space (planner can plan through it)
        # Use -1 for unknown if you want planner to treat it cautiously
        
        self.get_logger().info('Empty Global Map Node Started')
        self.get_logger().info(f'Map size: {self.width}x{self.height} cells')
        self.get_logger().info(f'Resolution: {self.resolution} m/cell')
        self.get_logger().info(f'Total area: {self.width*self.resolution}x{self.height*self.resolution} meters')
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Frame: {self.frame_id}')
        self.get_logger().info(f'Rolling window: {self.rolling_window}')

    def publish_map(self):
        """Publish empty global costmap"""
        map_msg = OccupancyGrid()
        
        # Header
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = self.frame_id
        
        # Map metadata
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        
        # Map origin (center on robot if rolling window)
        if self.rolling_window:
            # Center the map on the robot (origin at bottom-left)
            map_msg.info.origin.position.x = -(self.width * self.resolution) / 2.0
            map_msg.info.origin.position.y = -(self.height * self.resolution) / 2.0
        else:
            # Fixed origin at (0, 0)
            map_msg.info.origin.position.x = 0.0
            map_msg.info.origin.position.y = 0.0
        
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # Map data (empty/free space)
        map_msg.data = self.map_data.tolist()
        
        # Publish
        self.map_pub.publish(map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmptyGlobalMapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()