#!/usr/bin/env python3
"""
ROS2 Pure Pursuit Path Follower Node

Subscribes to:
  - /odom (nav_msgs/Odometry): Robot odometry
  - /planned_path (nav_msgs/Path): Path to follow

Publishes:
  - /cmd_vel (geometry_msgs/Twist): Velocity commands
  OR
  - /pwm_commands (custom message): PWM commands [surge, sway, yaw]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
from typing import List, Tuple, Optional

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray
import tf_transformations


class PurePursuitController:
    """Pure Pursuit path following algorithm"""
    
    def __init__(self, lookahead_distance: float = 1.0):
        self.lookahead = lookahead_distance
        self.waypoints = []
        self.current_waypoint_idx = 0
    
    def set_path(self, waypoints: List[Tuple[float, float]]):
        """Set waypoints from path"""
        self.waypoints = waypoints
        self.current_waypoint_idx = 0
    
    def get_target_point(self, robot_x: float, robot_y: float) -> Optional[Tuple[float, float]]:
        """Find lookahead point on path"""
        if not self.waypoints or self.current_waypoint_idx >= len(self.waypoints):
            return None
        
        if self.current_waypoint_idx >= len(self.waypoints) - 1:
            return self.waypoints[-1]
        
        target = self.waypoints[self.current_waypoint_idx]
        
        # Find point at lookahead distance
        for i in range(self.current_waypoint_idx, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            dist = np.sqrt((wx - robot_x)**2 + (wy - robot_y)**2)
            
            # Update current waypoint if we're close enough
            if dist < self.lookahead * 0.5 and i < len(self.waypoints) - 1:
                self.current_waypoint_idx = i + 1
            
            # Find point at lookahead distance
            if dist >= self.lookahead:
                target = (wx, wy)
                break
        
        # If near goal, target the goal
        goal_x, goal_y = self.waypoints[-1]
        dist_to_goal = np.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
        if dist_to_goal < self.lookahead:
            target = self.waypoints[-1]
        
        return target
    
    def compute_control_twist(self, robot_x: float, robot_y: float, robot_theta: float,
                             max_linear_vel: float = 1.0, max_angular_vel: float = 1.0) -> Twist:
        """
        Compute velocity commands (Twist message)
        For differential drive robots
        """
        cmd = Twist()
        
        if not self.waypoints:
            return cmd
        
        target = self.get_target_point(robot_x, robot_y)
        if target is None:
            return cmd
        
        target_x, target_y = target
        
        # Calculate error in robot frame
        dx = target_x - robot_x
        dy = target_y - robot_y
        
        # Transform to robot frame
        dx_body = dx * np.cos(-robot_theta) - dy * np.sin(-robot_theta)
        dy_body = dx * np.sin(-robot_theta) + dy * np.cos(-robot_theta)
        
        # Distance and angle to target
        dist = np.sqrt(dx_body**2 + dy_body**2)
        desired_angle = np.arctan2(dy_body, dx_body)
        
        # Linear velocity (proportional to distance)
        cmd.linear.x = min(max_linear_vel, max_linear_vel * (dist / self.lookahead))
        
        # Angular velocity (proportional to heading error)
        cmd.angular.z = np.clip(2.0 * desired_angle, -max_angular_vel, max_angular_vel)
        
        return cmd
    
    def compute_control_pwm(self, robot_x: float, robot_y: float, robot_theta: float,
                           max_surge: float = 1.0, max_sway: float = 0.5, 
                           max_yaw: float = 0.5) -> Tuple[float, float, float]:
        """
        Compute PWM commands for holonomic robot
        Returns: (surge_pwm, sway_pwm, yaw_pwm) in range [-1, 1]
        """
        if not self.waypoints:
            return (0.0, 0.0, 0.0)
        
        target = self.get_target_point(robot_x, robot_y)
        if target is None:
            return (0.0, 0.0, 0.0)
        
        target_x, target_y = target
        
        # Calculate error in robot frame
        dx = target_x - robot_x
        dy = target_y - robot_y
        
        # Transform to robot frame
        dx_body = dx * np.cos(-robot_theta) - dy * np.sin(-robot_theta)
        dy_body = dx * np.sin(-robot_theta) + dy * np.cos(-robot_theta)
        
        # Distance and angle to target
        dist = np.sqrt(dx_body**2 + dy_body**2)
        desired_angle = np.arctan2(dy_body, dx_body)
        
        # PWM commands
        surge_pwm = max_surge if dist > 0.5 else max_surge * 0.5
        sway_pwm = np.clip(dy_body * 0.5, -max_sway, max_sway)
        yaw_pwm = np.clip(desired_angle * 1.0, -max_yaw, max_yaw)
        
        return (surge_pwm, sway_pwm, yaw_pwm)
    
    def is_goal_reached(self, robot_x: float, robot_y: float, tolerance: float = 0.3) -> bool:
        """Check if robot reached the goal"""
        if not self.waypoints:
            return False
        
        goal_x, goal_y = self.waypoints[-1]
        dist = np.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
        
        return dist < tolerance


class PurePursuitNode(Node):
    """ROS2 Node for Pure Pursuit path following"""
    
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        # Parameters
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('use_pwm_control', False)  # True for PWM, False for Twist
        self.declare_parameter('max_surge_pwm', 0.8)
        self.declare_parameter('max_sway_pwm', 0.5)
        self.declare_parameter('max_yaw_pwm', 0.5)
        
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.use_pwm = self.get_parameter('use_pwm_control').value
        self.max_surge = self.get_parameter('max_surge_pwm').value
        self.max_sway = self.get_parameter('max_sway_pwm').value
        self.max_yaw = self.get_parameter('max_yaw_pwm').value
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )
        
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            qos_profile
        )
        
        # Publishers
        if self.use_pwm:
            self.pwm_pub = self.create_publisher(
                Float32MultiArray,
                '/pwm_commands',
                qos_profile
            )
            self.get_logger().info('Using PWM control mode')
        else:
            self.cmd_vel_pub = self.create_publisher(
                Twist,
                '/cmd_vel',
                qos_profile
            )
            self.get_logger().info('Using Twist control mode')
        
        # State variables
        self.current_pose = None
        self.current_orientation = None
        self.controller = PurePursuitController(self.lookahead)
        
        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_freq,
            self.control_callback
        )
        
        self.get_logger().info('Pure Pursuit Node initialized')
        self.get_logger().info(f'Lookahead distance: {self.lookahead} m')
        self.get_logger().info(f'Control frequency: {self.control_freq} Hz')
        self.get_logger().info(f'Goal tolerance: {self.goal_tolerance} m')
    
    def odom_callback(self, msg: Odometry):
        """Store current robot pose from odometry"""
        self.current_pose = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation
    
    def path_callback(self, msg: Path):
        """Receive new path and update controller"""
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return
        
        # Extract waypoints from path
        waypoints = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            waypoints.append((x, y))
        
        self.controller.set_path(waypoints)
        self.get_logger().info(f'Received new path with {len(waypoints)} waypoints')
    
    def get_yaw_from_quaternion(self, orientation) -> float:
        """Extract yaw angle from quaternion"""
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        euler = tf_transformations.euler_from_quaternion(quaternion)
        return euler[2]  # yaw
    
    def control_callback(self):
        """Main control loop"""
        # Check if we have pose data
        if self.current_pose is None or self.current_orientation is None:
            return
        
        # Check if we have a path
        if not self.controller.waypoints:
            return
        
        # Get current state
        robot_x = self.current_pose.x
        robot_y = self.current_pose.y
        robot_theta = self.get_yaw_from_quaternion(self.current_orientation)
        
        # Check if goal reached
        if self.controller.is_goal_reached(robot_x, robot_y, self.goal_tolerance):
            self.get_logger().info('Goal reached!', throttle_duration_sec=2.0)
            
            # Stop the robot
            if self.use_pwm:
                pwm_msg = Float32MultiArray()
                pwm_msg.data = [0.0, 0.0, 0.0]
                self.pwm_pub.publish(pwm_msg)
            else:
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
            
            return
        
        # Compute control commands
        if self.use_pwm:
            surge, sway, yaw = self.controller.compute_control_pwm(
                robot_x, robot_y, robot_theta,
                self.max_surge, self.max_sway, self.max_yaw
            )
            
            pwm_msg = Float32MultiArray()
            pwm_msg.data = [float(surge), float(sway), float(yaw)]
            self.pwm_pub.publish(pwm_msg)
            
        else:
            cmd_vel = self.controller.compute_control_twist(
                robot_x, robot_y, robot_theta,
                self.max_linear_vel, self.max_angular_vel
            )
            self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        if node.use_pwm:
            pwm_msg = Float32MultiArray()
            pwm_msg.data = [0.0, 0.0, 0.0]
            node.pwm_pub.publish(pwm_msg)
        else:
            stop_cmd = Twist()
            node.cmd_vel_pub.publish(stop_cmd)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()