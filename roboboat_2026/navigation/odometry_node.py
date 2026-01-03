#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry_node")

        # Constant Z value for 2D ground plane
        self.Z_COORDINATE = 0.0

        # Publisher for Odometry
        self.publisher_ = self.create_publisher(
            Odometry, 
            "/odom", 
            10
        )

        # Subscriber for GPS [lat, lon, heading]
        self.subscription = self.create_subscription(
            Float32MultiArray,
            "/GPS",
            self.gps_callback,
            10
        )

        # Service to manually reset the origin
        self.srv = self.create_service(Trigger, 'reset_origin', self.reset_origin_callback)

        # Origin coordinates (None until first GPS message received)
        self.lat0 = None
        self.lon0 = None
        
        self.get_logger().info(f"Odometry Node started. Z constant set to {self.Z_COORDINATE}.")

    def set_origin(self, lat, lon):
        """Defines the reference point for (0,0) local coordinates."""
        self.lat0 = lat
        self.lon0 = lon
        self.get_logger().info(f"--- ORIGIN SET --- Lat: {lat:.6f}, Lon: {lon:.6f}")

    def reset_origin_callback(self, request, response):
        """Allows resetting the origin via ROS service call."""
        self.lat0 = None
        self.lon0 = None
        response.success = True
        response.message = "Origin cleared. Next GPS message will set a new (0,0)."
        return response
    
    def degrees_to_radians(self, degrees):
        """Converts degrees to radians."""
        return degrees * (math.pi / 180.0)

    def get_quaternion_from_euler(self, yaw):
        """Converts yaw (radians) to a ROS 2 quaternion."""
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }

    def latlon_to_xy_meters(self, lat, lon):
        """Standard Equirectangular projection for small local areas."""
        R = 6371000 # Earth radius in meters
        phi = math.radians(lat)
        lmd = math.radians(lon)
        phi0 = math.radians(self.lat0)
        lmd0 = math.radians(self.lon0)
        
        # x = Easting, y = Northing
        x = R * math.cos(phi0) * (lmd - lmd0)
        y = R * (phi - phi0)
        return x, y

    def gps_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            self.get_logger().warn("GPS message needs [lat, lon, heading]")
            return

        lat, lon, heading = msg.data[0], msg.data[1], msg.data[2]

        # Handle origin initialization
        if self.lat0 is None or self.lon0 is None:
            self.set_origin(lat, lon)

        # Coordinate conversion
        x, y = self.latlon_to_xy_meters(lat, lon)
        
        # Initialize Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set Position (Z is constant)
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = self.Z_COORDINATE

        # Set Orientation
        yaw = self.degrees_to_radians(heading)
        q = self.get_quaternion_from_euler(yaw)
        odom_msg.pose.pose.orientation.x = q['x']
        odom_msg.pose.pose.orientation.y = q['y']
        odom_msg.pose.pose.orientation.z = q['z']
        odom_msg.pose.pose.orientation.w = q['w']

        # Add small covariance to prevent errors in filters
        # 6x6 matrix for Pose [x, y, z, roll, pitch, yaw]
        odom_msg.pose.covariance[0] = 0.1  # x
        odom_msg.pose.covariance[7] = 0.1  # y
        odom_msg.pose.covariance[35] = 0.05 # yaw

        self.publisher_.publish(odom_msg)
        self.get_logger().info(f"Odom: x={x:.2f}, y={y:.2f}, z={self.Z_COORDINATE}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()