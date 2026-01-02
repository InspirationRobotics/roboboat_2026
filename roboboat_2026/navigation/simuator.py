#!/usr/bin/env python3
"""
Vehicle Simulator Node
Simulates a vehicle responding to cmd_vel commands and publishes odometry
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Twist, Pose
from scipy.spatial.transform import Rotation
import numpy as np


class VehicleSimulator(Node):
    """Simulates a differential drive vehicle"""
    
    def __init__(self):
        super().__init__('vehicle_simulator_node')
        
        # Parameters
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('update_rate', 50.0)  # Hz
        self.declare_parameter('noise_linear', 0.02)  # Velocity noise std dev
        self.declare_parameter('noise_angular', 0.01)
        self.declare_parameter('noise_position', 0.05)  # Position noise std dev
        self.declare_parameter('map_size', 100.0)  # Map size in meters
        self.declare_parameter('map_resolution', 0.5)  # Map resolution in meters/cell
        
        initial_x = self.get_parameter('initial_x').value
        initial_y = self.get_parameter('initial_y').value
        initial_yaw = self.get_parameter('initial_yaw').value
        update_rate = self.get_parameter('update_rate').value
        self.noise_linear = self.get_parameter('noise_linear').value
        self.noise_angular = self.get_parameter('noise_angular').value
        self.noise_position = self.get_parameter('noise_position').value
        map_size = self.get_parameter('map_size').value
        map_resolution = self.get_parameter('map_resolution').value
        
        # Vehicle state [x, y, yaw, vx, vy, vyaw]
        self.state = np.array([initial_x, initial_y, initial_yaw, 0.0, 0.0, 0.0])
        
        # Command velocities
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/fused/odometry', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/local_map', 10)
        
        # Generate static map with obstacles
        self.occupancy_grid = self.generate_test_map(map_size, map_resolution)
        self.map_resolution = map_resolution
        self.map_size = map_size
        
        # Simulation timer
        dt = 1.0 / update_rate
        self.dt = dt
        self.timer = self.create_timer(dt, self.simulate_step)
        
        # Map publishing timer (publish less frequently)
        self.map_timer = self.create_timer(2.0, self.publish_map)
        
        self.get_logger().info('Vehicle Simulator Node Started')
        self.get_logger().info(f'Initial pose: x={initial_x}, y={initial_y}, yaw={initial_yaw}')
        self.get_logger().info(f'Update rate: {update_rate} Hz')
        self.get_logger().info(f'Map size: {map_size}m, resolution: {map_resolution}m/cell')
    
    def generate_test_map(self, map_size, resolution):
        """Generate a test occupancy grid with obstacles"""
        # Grid dimensions
        grid_size = int(map_size / resolution)
        grid = np.zeros((grid_size, grid_size), dtype=np.int8)
        
        # Add some rectangular obstacles
        obstacles = [
            # (center_x, center_y, width, height)
            (25, 25, 8, 8),
            (50, 20, 6, 20),
            (30, 45, 15, 5),
            (70, 60, 10, 10),
            (15, 70, 20, 8),
        ]
        
        origin_x = -map_size / 2
        origin_y = -map_size / 2
        
        for obs_x, obs_y, obs_w, obs_h in obstacles:
            # Convert to grid coordinates
            gx = int((obs_x - origin_x) / resolution)
            gy = int((obs_y - origin_y) / resolution)
            gw = int(obs_w / resolution)
            gh = int(obs_h / resolution)
            
            # Fill obstacle
            x_start = max(0, gx - gw // 2)
            x_end = min(grid_size, gx + gw // 2)
            y_start = max(0, gy - gh // 2)
            y_end = min(grid_size, gy + gh // 2)
            
            grid[y_start:y_end, x_start:x_end] = 100
        
        # Add border walls
        grid[0:2, :] = 100  # Bottom
        grid[-2:, :] = 100  # Top
        grid[:, 0:2] = 100  # Left
        grid[:, -2:] = 100  # Right
        
        return grid
    
    def cmd_vel_callback(self, msg):
        """Receive velocity commands"""
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z
    
    def simulate_step(self):
        """Update vehicle state using simple kinematic model"""
        # Add noise to commands
        noisy_linear = self.cmd_linear + np.random.normal(0, self.noise_linear)
        noisy_angular = self.cmd_angular + np.random.normal(0, self.noise_angular)
        
        # Current state
        x, y, yaw, vx, vy, vyaw = self.state
        
        # Update velocities (simple first-order dynamics)
        alpha = 0.7  # Smoothing factor
        vx = alpha * vx + (1 - alpha) * noisy_linear
        vyaw = alpha * vyaw + (1 - alpha) * noisy_angular
        
        # Update pose using kinematic model (differential drive)
        x += vx * np.cos(yaw) * self.dt
        y += vx * np.sin(yaw) * self.dt
        yaw += vyaw * self.dt
        
        # Normalize yaw
        yaw = np.arctan2(np.sin(yaw), np.cos(yaw))
        
        # Add position noise
        x += np.random.normal(0, self.noise_position) * self.dt
        y += np.random.normal(0, self.noise_position) * self.dt
        
        # Update state
        self.state = np.array([x, y, yaw, vx, vy, vyaw])
        
        # Publish odometry
        self.publish_odometry()
    
    def publish_odometry(self):
        """Publish current odometry"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        
        # Pose
        odom.pose.pose.position.x = self.state[0]
        odom.pose.pose.position.y = self.state[1]
        odom.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        r = Rotation.from_euler('z', self.state[2])
        quat = r.as_quat()
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # Velocity (in base_link frame for differential drive)
        odom.twist.twist.linear.x = self.state[3]
        odom.twist.twist.linear.y = self.state[4]
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.state[5]
        
        # Covariances (diagonal)
        pose_cov = np.zeros(36)
        pose_cov[0] = 0.1   # x
        pose_cov[7] = 0.1   # y
        pose_cov[35] = 0.05  # yaw
        odom.pose.covariance = pose_cov.tolist()
        
        twist_cov = np.zeros(36)
        twist_cov[0] = 0.05  # vx
        twist_cov[7] = 0.05  # vy
        twist_cov[35] = 0.02  # vyaw
        odom.twist.covariance = twist_cov.tolist()
        
        self.odom_pub.publish(odom)
    
    def publish_map(self):
        """Publish the occupancy grid map"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        # Map metadata
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.occupancy_grid.shape[1]
        map_msg.info.height = self.occupancy_grid.shape[0]
        
        # Origin (bottom-left corner)
        map_msg.info.origin.position.x = -self.map_size / 2
        map_msg.info.origin.position.y = -self.map_size / 2
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # Flatten grid data
        map_msg.data = self.occupancy_grid.flatten().tolist()
        
        self.map_pub.publish(map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()