#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
import numpy as np
from scipy.spatial.transform import Rotation


class EKF:
    """Extended Kalman Filter for 2D pose and velocity estimation"""
    
    def __init__(self):
        # State vector: [x, y, vx, vy, yaw]
        self.state = np.zeros(5)
        
        # Covariance matrix
        self.P = np.eye(5) * 1.0
        
        # Process noise
        self.Q = np.eye(5) * 0.01
        self.Q[2:4, 2:4] *= 0.1  # Higher noise for velocities
        self.Q[4, 4] *= 0.05  # Yaw noise
        
        # Measurement noise
        self.R_gps = np.eye(2) * 5.0  # GPS position noise (meters)
    
    def predict(self, dt):
        """Prediction step with constant velocity model"""
        # State transition matrix
        F = np.eye(5)
        F[0, 2] = dt  # x = x + vx * dt
        F[1, 3] = dt  # y = y + vy * dt
        
        # Predict state
        self.state = F @ self.state
        
        # Normalize yaw to [-pi, pi]
        self.state[4] = np.arctan2(np.sin(self.state[4]), np.cos(self.state[4]))
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
    
    def update_position(self, position):
        """Update step with GPS position measurement"""
        # Measurement matrix (GPS measures x, y position only)
        H = np.zeros((2, 5))
        H[0, 0] = 1  # Measure x
        H[1, 1] = 1  # Measure y
        
        # Innovation
        z = position
        z_pred = H @ self.state
        y = z - z_pred
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_gps
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        
        # Update covariance
        self.P = (np.eye(5) - K @ H) @ self.P
    
    def update_velocity(self, velocity, alpha=0.3):
        """Update velocity using complementary filter"""
        self.state[2] = alpha * velocity[0] + (1 - alpha) * self.state[2]
        self.state[3] = alpha * velocity[1] + (1 - alpha) * self.state[3]
    
    def update_yaw(self, yaw, alpha=0.8):
        """Update yaw using complementary filter"""
        yaw_diff = np.arctan2(np.sin(yaw - self.state[4]), np.cos(yaw - self.state[4]))
        self.state[4] = self.state[4] + alpha * yaw_diff
        
        # Normalize yaw to [-pi, pi]
        self.state[4] = np.arctan2(np.sin(self.state[4]), np.cos(self.state[4]))
    
    def get_state(self):
        """Return current state [x, y, vx, vy, yaw]"""
        return self.state.copy()
    
    def get_covariance(self):
        """Return current covariance matrix"""
        return self.P.copy()


class GPSFusion(Node):
    """ROS2 Node for Differential GPS sensor fusion"""
    
    def __init__(self):
        super().__init__('gps_fusion_node')
        
        # Initialize EKF
        self.ekf = EKF()
        
        # Subscriber
        self.gps_sub = self.create_subscription(
            Float32MultiArray,
            '/GPS',  # TODO: Replace with your GPS topic
            self.gps_callback,
            10
        )
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/fused/pose', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, '/fused/velocity', 10)
        self.odom_pub = self.create_publisher(Odometry, '/fused/odometry', 10)
        self.origin_pub = self.create_publisher(Float32MultiArray, 'ekforigin',10)
        self.declare_parameter('origin', Parameter.Type.DOUBLE_ARRAY)
        
        # Reference GPS coordinates (set on first GPS message)
        self.gps_origin = None
        self.last_time = None
        
        # Previous GPS position for velocity estimation
        self.prev_gps_pos = None
        self.prev_gps_time = None
        
        self.get_logger().info('GPS Fusion Node Started')
        self.get_logger().info('Waiting for GPS data...')
        self.get_logger().info('GPS Topic: /gps/data')
        self.get_logger().info('Expected format: [lat, lon, heading]')
    
    def gps_to_local(self, lat, lon):
        """Convert GPS coordinates to local ENU frame (2D)"""
        if self.gps_origin is None:
            self.gps_origin = (lat, lon)
            self.get_logger().info(f"GPS origin at lat -> {lat}, long -> {lon}")
            self.set_parameters([
                rclpy.parameter.Parameter(
                    'origin',
                    rclpy.Parameter.Type.DOUBLE_ARRAY,
                    [lat, lon]
                )
            ])
            return np.array([0.0, 0.0])
        
        lat0, lon0 = self.gps_origin
        
        # Approximate conversion (for small distances)
        R_earth = 6378137.0  # Earth radius in meters
        
        dlat = np.radians(lat - lat0)
        dlon = np.radians(lon - lon0)
        
        x = R_earth * dlon * np.cos(np.radians(lat0))
        y = R_earth * dlat
        
        return np.array([x, y])
    
    def heading_to_yaw(self, heading_deg):
        """Convert GPS heading (0=North, 90=East) to yaw (0=East, 90=North)
        
        GPS heading: North=0°, East=90°, South=180°, West=270°
        ROS yaw: East=0°, North=90°, West=180°/-180°, South=-90°
        """
        # Convert to radians and adjust frame
        yaw_rad = np.radians(90.0 - heading_deg)
        
        # Normalize to [-pi, pi]
        yaw_rad = np.arctan2(np.sin(yaw_rad), np.cos(yaw_rad))
        
        return yaw_rad
    
    def estimate_velocity_from_gps(self, position, timestamp):
        """Estimate velocity from consecutive GPS positions"""
        if self.prev_gps_pos is not None and self.prev_gps_time is not None:
            dt = (timestamp - self.prev_gps_time).nanoseconds / 1e9
            if dt > 0 and dt < 2.0:  # Sanity check
                vel = (position - self.prev_gps_pos) / dt
                return vel
        return None
    
    def gps_callback(self, msg):
        """Handle GPS messages"""
        # Unpack GPS data
        if len(msg.data) < 3:
            self.get_logger().warn('GPS message does not contain enough data')
            return
        
        lat, lon, heading = msg.data[0], msg.data[1], msg.data[2]
        
        current_time = self.get_clock().now()
        
        # Convert GPS to local coordinates
        position = self.gps_to_local(lat, lon)
            
        
        # Convert heading to yaw
        yaw = self.heading_to_yaw(heading)
        
        # Estimate velocity from GPS
        vel_estimate = self.estimate_velocity_from_gps(position, current_time)
        if vel_estimate is not None:
            self.ekf.update_velocity(vel_estimate)
        
        # Store for next velocity estimate
        self.prev_gps_pos = position
        self.prev_gps_time = current_time
        
        # Prediction step
        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            if dt > 0 and dt < 1.0:  # Sanity check
                self.ekf.predict(dt)
        
        self.last_time = current_time
        
        # Update with GPS measurements
        self.ekf.update_position(position)
        self.ekf.update_yaw(yaw)
        
        # Publish fused data
        self.publish_fused_data(current_time)
        
        origin_msg = Float32MultiArray()
        origin_msg.data = self.gps_origin
        # print(self.gps_origin)
        self.origin_pub.publish(origin_msg)
    
    def publish_fused_data(self, timestamp):
        """Publish fused pose and velocity"""
        state = self.ekf.get_state()
        covariance = self.ekf.get_covariance()
        
        # Publish Pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp.to_msg()
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.position.x = state[0]
        pose_msg.pose.position.y = state[1]
        pose_msg.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        r = Rotation.from_euler('z', state[4])
        quat = r.as_quat()
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        self.pose_pub.publish(pose_msg)
        
        # Publish Velocity
        vel_msg = TwistStamped()
        vel_msg.header.stamp = timestamp.to_msg()
        vel_msg.header.frame_id = 'map'
        
        vel_msg.twist.linear.x = state[2]
        vel_msg.twist.linear.y = state[3]
        vel_msg.twist.linear.z = 0.0
        vel_msg.twist.angular.z = 0.0
        
        self.velocity_pub.publish(vel_msg)
        
        # Publish Odometry (combined pose + velocity)
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose = pose_msg.pose
        odom_msg.twist.twist = vel_msg.twist
        
        # Set covariances
        odom_msg.pose.covariance = np.zeros(36)
        odom_msg.pose.covariance[0] = covariance[0, 0]  # x
        odom_msg.pose.covariance[7] = covariance[1, 1]  # y
        odom_msg.pose.covariance[35] = covariance[4, 4]  # yaw
        
        odom_msg.twist.covariance = np.zeros(36)
        odom_msg.twist.covariance[0] = covariance[2, 2]  # vx
        odom_msg.twist.covariance[7] = covariance[3, 3]  # vy
        
        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GPSFusion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
