#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from sensor_msgs_py import point_cloud2
import numpy as np

class LidarCostmapNode(Node):
    def __init__(self):
        super().__init__('lidar_costmap_node')
        
        # Declare parameters with defaults
        self.declare_parameter('map_resolution', 0.1)  # meters per cell
        self.declare_parameter('map_width', 100)       # cells
        self.declare_parameter('map_height', 100)      # cells
        self.declare_parameter('density_threshold', 1) # min points per cell to mark as occupied
        self.declare_parameter('max_cost', 100)        # maximum occupancy cost
        self.declare_parameter('cost_scaling_factor', 10.0)  # scale point density to cost
        self.declare_parameter('update_rate', 10.0)   # Hz

        # Get parameters
        self.resolution = self.get_parameter('map_resolution').value
        self.width = self.get_parameter('map_width').value
        self.height = self.get_parameter('map_height').value
        self.density_threshold = self.get_parameter('density_threshold').value
        self.max_cost = self.get_parameter('max_cost').value
        self.cost_scaling = self.get_parameter('cost_scaling_factor').value
        self.update_rate = self.get_parameter('update_rate').value

        # Initialize latest point cloud
        self.latest_points = None

        # Subscriber
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.pointcloud_callback,
            10
        )

        # Publisher
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/local_costmap',
            10
        )

        # Timer for periodic costmap updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_costmap)

        # Initialize costmap data
        self.costmap_data = np.zeros((self.height, self.width), dtype=np.int8)

        self.get_logger().info('Lidar Costmap Node Started')
        self.get_logger().info(f'Map size: {self.width}x{self.height} cells')
        self.get_logger().info(f'Resolution: {self.resolution} m/cell')
        self.get_logger().info(f'Density threshold: {self.density_threshold} points/cell')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')

    def pointcloud_callback(self, msg):
        """Store latest point cloud data"""
        points = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            # Optionally ignore points outside robot height range
            # if 0.0 <= z <= 2.0:
            #     points.append([x, y])
            points.append([x, y])

        if points:
            self.latest_points = np.array(points)
        else:
            self.latest_points = None

    def publish_costmap(self):
        """Generate and publish robot-centered costmap"""
        if self.latest_points is None:
            self.get_logger().warn('No point cloud data available', throttle_duration_sec=5.0)
            return

        # Reset costmap
        self.costmap_data.fill(0)

        # Density grid to count points per cell
        density_grid = np.zeros((self.height, self.width), dtype=np.int32)

        # Robot-centered map origin (bottom-left corner)
        origin_x = - (self.width * self.resolution) / 2.0
        origin_y = - (self.height * self.resolution) / 2.0

        # Convert points to grid indices
        for point in self.latest_points:
            world_x, world_y = point
            grid_x = int((world_x - origin_x) / self.resolution)
            grid_y = int((world_y - origin_y) / self.resolution)

            if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                density_grid[grid_y, grid_x] += 1

        # Convert density to occupancy cost
        self.costmap_data = np.clip(density_grid * self.cost_scaling, 0, self.max_cost)
        self.costmap_data[density_grid < self.density_threshold] = 0

        # Build OccupancyGrid message
        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = 'livox_frame'  # robot-centered

        # Map info
        costmap_msg.info.resolution = self.resolution
        costmap_msg.info.width = self.width
        costmap_msg.info.height = self.height
        costmap_msg.info.origin.position.x = origin_x
        costmap_msg.info.origin.position.y = origin_y
        costmap_msg.info.origin.position.z = 0.0
        costmap_msg.info.origin.orientation.w = 1.0

        # Flatten row-major
        # Scale and clip density to valid int8 values
        costmap = np.clip(density_grid * self.cost_scaling, 0, 127).astype(np.int8)
        self.costmap_data = costmap
        costmap_msg.data = self.costmap_data.flatten().tolist()


        # Publish
        self.costmap_pub.publish(costmap_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarCostmapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
