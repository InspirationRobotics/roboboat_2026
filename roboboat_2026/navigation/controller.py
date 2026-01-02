#!/usr/bin/env python3
"""
Pure Pursuit Controller Node
Implements pure pursuit algorithm for path following
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PointStamped
from scipy.spatial.transform import Rotation
import numpy as np


class PurePursuitNode(Node):
    """Pure Pursuit path following controller"""
    
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        # Parameters
        self.declare_parameter('lookahead_distance', 5.0)
        self.declare_parameter('lookahead_gain', 0.5)
        self.declare_parameter('max_linear_velocity', 2.0)
        self.declare_parameter('min_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('wheelbase', 1.0)  # For Ackermann steering
        
        self.base_lookahead = self.get_parameter('lookahead_distance').value
        self.lookahead_gain = self.get_parameter('lookahead_gain').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.min_linear_vel = self.get_parameter('min_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.wheelbase = self.get_parameter('wheelbase').value
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/fused/odometry',
            self.odom_callback,
            10
        )
        
        self.path_sub = self.create_subscription(
            Path,
            '/global_path',
            self.path_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lookahead_pub = self.create_publisher(PointStamped, '/lookahead_point', 10)
        
        # State
        self.current_pose = None
        self.current_velocity = 0.0
        self.path = None
        self.goal_reached = False
        
        # Control timer
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        
        self.get_logger().info('Pure Pursuit Controller Node Started')
        self.get_logger().info(f'Base lookahead: {self.base_lookahead}m')
        self.get_logger().info(f'Max velocity: {self.max_linear_vel}m/s')
    
    def odom_callback(self, msg):
        """Update current pose from odometry"""
        self.current_pose = msg.pose.pose
        
        # Calculate current speed
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_velocity = np.sqrt(vx**2 + vy**2)
    
    def path_callback(self, msg):
        """Receive new path"""
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return
        
        self.path = msg
        self.goal_reached = False
        self.get_logger().info(f'Received new path with {len(msg.poses)} waypoints')
    
    def get_yaw_from_quaternion(self, quat):
        """Extract yaw from quaternion"""
        r = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        _, _, yaw = r.as_euler('xyz')
        return yaw
    
    def find_lookahead_point(self):
        """Find the lookahead point on the path"""
        if self.current_pose is None or self.path is None:
            return None
        
        # Adaptive lookahead distance based on velocity
        lookahead_dist = self.base_lookahead + self.lookahead_gain * self.current_velocity
        
        # Current position
        cx = self.current_pose.position.x
        cy = self.current_pose.position.y
        
        # Find closest point on path
        min_dist = float('inf')
        closest_idx = 0
        
        for i, pose in enumerate(self.path.poses):
            dx = pose.pose.position.x - cx
            dy = pose.pose.position.y - cy
            dist = np.sqrt(dx**2 + dy**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Search forward from closest point for lookahead point
        for i in range(closest_idx, len(self.path.poses)):
            px = self.path.poses[i].pose.position.x
            py = self.path.poses[i].pose.position.y
            
            dx = px - cx
            dy = py - cy
            dist = np.sqrt(dx**2 + dy**2)
            
            if dist >= lookahead_dist:
                return np.array([px, py]), i
        
        # If no point found, return the last point
        last_pose = self.path.poses[-1].pose
        return np.array([last_pose.position.x, last_pose.position.y]), len(self.path.poses) - 1
    
    def compute_steering_angle(self, lookahead_point):
        """Compute steering angle using pure pursuit"""
        if self.current_pose is None:
            return 0.0
        
        # Transform lookahead point to vehicle frame
        cx = self.current_pose.position.x
        cy = self.current_pose.position.y
        cyaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Vector to lookahead point in global frame
        dx = lookahead_point[0] - cx
        dy = lookahead_point[1] - cy
        
        # Transform to vehicle frame
        lx = np.cos(cyaw) * dx + np.sin(cyaw) * dy
        ly = -np.sin(cyaw) * dx + np.cos(cyaw) * dy
        
        # Pure pursuit algorithm
        ld = np.sqrt(lx**2 + ly**2)
        
        if ld < 0.1:  # Avoid division by zero
            return 0.0
        
        # Curvature
        alpha = np.arctan2(ly, lx)
        curvature = 2.0 * np.sin(alpha) / ld
        
        # Steering angle (for differential drive, this becomes angular velocity)
        # For Ackermann: steering_angle = np.arctan(curvature * self.wheelbase)
        
        return curvature
    
    def check_goal_reached(self):
        """Check if vehicle has reached the goal"""
        if self.current_pose is None or self.path is None:
            return False
        
        goal = self.path.poses[-1].pose
        dx = goal.position.x - self.current_pose.position.x
        dy = goal.position.y - self.current_pose.position.y
        dist = np.sqrt(dx**2 + dy**2)
        
        return dist < self.goal_tolerance
    
    def control_loop(self):
        """Main control loop"""
        if self.current_pose is None or self.path is None:
            return
        
        # Check if goal reached
        if self.check_goal_reached():
            if not self.goal_reached:
                self.get_logger().info('Goal reached!')
                self.goal_reached = True
            
            # Stop the vehicle
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return
        
        # Find lookahead point
        result = self.find_lookahead_point()
        if result is None:
            return
        
        lookahead_point, _ = result
        
        # Publish lookahead point for visualization
        lookahead_msg = PointStamped()
        lookahead_msg.header.frame_id = 'map'
        lookahead_msg.header.stamp = self.get_clock().now().to_msg()
        lookahead_msg.point.x = lookahead_point[0]
        lookahead_msg.point.y = lookahead_point[1]
        lookahead_msg.point.z = 0.0
        self.lookahead_pub.publish(lookahead_msg)
        
        # Compute steering
        curvature = self.compute_steering_angle(lookahead_point)
        
        # Compute velocities
        cmd = Twist()
        
        # Linear velocity (reduce speed on sharp turns)
        speed_factor = 1.0 / (1.0 + abs(curvature) * 2.0)
        cmd.linear.x = max(self.min_linear_vel, self.max_linear_vel * speed_factor)
        
        # Angular velocity
        cmd.angular.z = np.clip(
            curvature * cmd.linear.x,
            -self.max_angular_vel,
            self.max_angular_vel
        )
        
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()