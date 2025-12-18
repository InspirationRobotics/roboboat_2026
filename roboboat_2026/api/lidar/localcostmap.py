#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point
from sensor_msgs_py import point_cloud2
import numpy as np

class LidarCostmapNode(Node):
    def __init__(self):
        super().__init__('lidar_costmap_node')
        
        # Declare parameters with defaults
        self.declare_parameter('map_resolution', 0.1)  # meters per cell
        self.declare_parameter('map_width', 100)  # cells
        self.declare_parameter('map_height', 100)  # cells
        self.declare_parameter('density_threshold', 1)  # min points per cell to mark as occupied
        self.declare_parameter('max_cost', 100)  # maximum occupancy cost
        self.declare_parameter('cost_scaling_factor', 10.0)  # scale point density to cost
        self.declare_parameter('update_rate', 10.0)  # Hz
        
        # Get parameters
        self.resolution = self.get_parameter('map_resolution').value
        self.width = self.get_parameter('map_width').value
        self.height = self.get_parameter('map_height').value
        self.density_threshold = self.get_parameter('density_threshold').value
        self.max_cost = self.get_parameter('max_cost').value
        self.cost_scaling = self.get_parameter('cost_scaling_factor').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Subscribers
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/lidar_2d',
            self.pointcloud_callback,
            10
        )
        
        self.gps_sub = self.create_subscription(
            Pose,
            '/GPS/map',
            self.gps_callback,
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
        
        # Storage
        self.latest_points = None
        self.robot_position = Point(x=0.0, y=0.0, z=0.0)  # Default position
        self.robot_yaw = 0.0
        self.gps_received = False
        
        # Initialize costmap data
        self.costmap_data = np.zeros((self.height, self.width), dtype=np.int8)
        
        self.get_logger().info('Lidar Costmap Node Started')
        self.get_logger().info(f'Map size: {self.width}x{self.height} cells')
        self.get_logger().info(f'Resolution: {self.resolution} m/cell')
        self.get_logger().info(f'Density threshold: {self.density_threshold} points/cell')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
    
    def gps_callback(self, msg):
        """Store robot position from GPS/map topic"""
        self.robot_position = msg.position
        
        # Extract yaw from quaternion if needed for future use
        # For now, we assume the robot is aligned with the map frame
        self.gps_received = True
        
        self.get_logger().debug(
            f'Robot position updated: ({self.robot_position.x:.2f}, '
            f'{self.robot_position.y:.2f})'
        )
    
    def pointcloud_callback(self, msg):
        """Store latest point cloud data"""
        points = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1]])
        
        if len(points) > 0:
            self.latest_points = np.array(points)
        else:
            self.latest_points = None
    
    def publish_costmap(self):
        """Generate and publish costmap from latest point cloud"""
        if self.latest_points is None:
            self.get_logger().warn('No point cloud data available', throttle_duration_sec=5.0)
            return
        
        if not self.gps_received:
            self.get_logger().warn('No GPS position received yet', throttle_duration_sec=5.0)
            return
        
        # Reset costmap
        self.costmap_data.fill(0)
        
        # Create density grid to count points per cell
        density_grid = np.zeros((self.height, self.width), dtype=np.int32)
        
        # Map origin is centered on robot
        origin_x = self.robot_position.x - (self.width * self.resolution) / 2.0
        origin_y = self.robot_position.y - (self.height * self.resolution) / 2.0
        
        # Convert world points to grid coordinates
        for point in self.latest_points:
            # Point is in robot frame, transform to map frame
            world_x = self.robot_position.x + point[0]
            world_y = self.robot_position.y + point[1]
            
            # Convert to grid coordinates
            grid_x = int((world_x - origin_x) / self.resolution)
            grid_y = int((world_y - origin_y) / self.resolution)
            
            # Check bounds
            if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                density_grid[grid_y, grid_x] += 1
        
        # Convert density to cost
        for i in range(self.height):
            for j in range(self.width):
                if density_grid[i, j] >= self.density_threshold:
                    # Scale density to cost value
                    cost = min(int(density_grid[i, j] * self.cost_scaling), self.max_cost)
                    self.costmap_data[i, j] = cost
                else:
                    # Free space
                    self.costmap_data[i, j] = 0
        
        # Create OccupancyGrid message
        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = 'map'
        
        # Map metadata
        costmap_msg.info.resolution = self.resolution
        costmap_msg.info.width = self.width
        costmap_msg.info.height = self.height
        
        # Origin is at bottom-left corner of the map
        costmap_msg.info.origin.position.x = origin_x
        costmap_msg.info.origin.position.y = origin_y
        costmap_msg.info.origin.position.z = 0.0
        costmap_msg.info.origin.orientation.w = 1.0
        
        # Flatten the costmap data (row-major order)
        costmap_msg.data = self.costmap_data.flatten().tolist()
        
        # Publish
        self.costmap_pub.publish(costmap_msg)
        
        occupied_cells = np.sum(density_grid >= self.density_threshold)
        self.get_logger().debug(
            f'Published costmap with {occupied_cells} occupied cells '
            f'({len(self.latest_points)} points)'
        )

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