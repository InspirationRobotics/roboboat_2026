#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2
import numpy as np
from scipy.spatial.transform import Rotation
import struct

class Lidar2DProjection(Node):
    def __init__(self):
        super().__init__('lidar_2d_projection')
        
        # Subscribers
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.lidar_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/oak/imu/data',
            self.imu_callback,
            10
        )
        
        # Publisher for 2D point cloud
        self.pc2d_pub = self.create_publisher(
            PointCloud2,
            '/lidar_2d',
            10
        )
        
        # Store latest IMU data
        self.latest_roll = 0.0
        self.latest_pitch = 0.0
        self.imu_received = False
        
        self.get_logger().info('Lidar 2D Projection Node Started')
        self.get_logger().info('Subscribing to /livox/lidar and /oak/imu/data')
        self.get_logger().info('Publishing to /lidar_2d')
    
    def imu_callback(self, msg):
        """Extract roll and pitch from IMU orientation quaternion"""
        # Convert quaternion to euler angles
        quat = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        
        # Convert to rotation object
        rot = Rotation.from_quat(quat)
        euler = rot.as_euler('xyz', degrees=False)  # Roll, Pitch, Yaw
        
        self.latest_roll = 0# euler[0]
        self.latest_pitch = 0 #euler[1]
        self.imu_received = True
    
    def lidar_callback(self, msg):
        """Process 3D point cloud and project to 2D"""
        if not self.imu_received:
            self.get_logger().warn('No IMU data received yet, skipping frame')
            return
        
        # Read point cloud data
        points = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])
        
        if len(points) == 0:
            self.get_logger().warn('Empty point cloud received')
            return
        
        points = np.array(points)
        
        # Create rotation matrix to correct for pitch and roll
        # We want to rotate the point cloud to compensate for the sensor tilt
        R_roll = Rotation.from_euler('x', -self.latest_roll)
        R_pitch = Rotation.from_euler('y', -self.latest_pitch)
        R_combined = R_pitch * R_roll
        
        # Apply rotation to all points
        points_corrected = R_combined.apply(points)
        
        # Project to 2D by taking X and Y, setting Z to 0
        points_2d = np.zeros_like(points_corrected)
        points_2d[:, 0] = points_corrected[:, 0]  # X
        points_2d[:, 1] = points_corrected[:, 1]  # Y
        points_2d[:, 2] = 0.0                      # Z = 0 (ground plane)
        
        # Create new PointCloud2 message
        header = msg.header
        header.frame_id = msg.header.frame_id  # Keep original frame or change to 'base_link'
        
        # Create point cloud data
        points_2d_msg = self.create_point_cloud2(header, points_2d)
        
        # Publish
        self.pc2d_pub.publish(points_2d_msg)
        
        self.get_logger().debug(
            f'Processed {len(points)} points. Roll: {np.degrees(self.latest_roll):.2f}°, '
            f'Pitch: {np.degrees(self.latest_pitch):.2f}°'
        )
    
    def create_point_cloud2(self, header, points):
        """Create a PointCloud2 message from numpy array"""
        # Create the fields for x, y, z
        fields = [
            point_cloud2.PointField(
                name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1
            ),
            point_cloud2.PointField(
                name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1
            ),
            point_cloud2.PointField(
                name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1
            ),
        ]
        
        # Convert points to bytes
        cloud_data = []
        for point in points:
            cloud_data.append(struct.pack('fff', point[0], point[1], point[2]))
        
        # Create PointCloud2 message
        pc2_msg = PointCloud2()
        pc2_msg.header = header
        pc2_msg.height = 1
        pc2_msg.width = len(points)
        pc2_msg.fields = fields
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 12  # 3 floats * 4 bytes
        pc2_msg.row_step = pc2_msg.point_step * len(points)
        pc2_msg.is_dense = True
        pc2_msg.data = b''.join(cloud_data)
        
        return pc2_msg

def main(args=None):
    rclpy.init(args=args)
    node = Lidar2DProjection()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()