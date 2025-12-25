#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger # Service to trigger an origin reset

class OdometryNode(Node):
    def __init__(self):
        super().__init__("OdometryNode")

        # Publisher for local X, Y coordinates
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 
            "/local_xy", 
            10
        )

        # Subscriber for GPS
        self.subscription = self.create_subscription(
            Float32MultiArray,
            "/GPS",
            self.gps_callback,
            10
        )

        # Service to manually reset the origin to the next GPS point
        self.srv = self.create_service(Trigger, 'reset_origin', self.reset_origin_callback)

        # Variables to store the origin
        self.lat0 = None
        self.lon0 = None
        
        self.get_logger().info("Odometry Node started. The first GPS fix will set the (0,0) origin.")

    def set_origin(self, lat, lon):
        """
        Sets the reference point for all future XY conversions.
        """
        self.lat0 = lat
        self.lon0 = lon
        self.get_logger().info(f"--- ORIGIN SET --- Lat: {lat:.6f}, Lon: {lon:.6f} -> (0.0, 0.0)")

    def reset_origin_callback(self, request, response):
        """
        Allows resetting the origin via ROS service: 'ros2 service call /reset_origin std_srvs/srv/Trigger'
        """
        self.lat0 = None
        self.lon0 = None
        response.success = True
        response.message = "Origin cleared. Next GPS message will set a new (0,0)."
        return response

    def latlon_to_xy_meters(self, lat, lon):
        """
        Convert GPS (lat, lon) to local x,y meters (East, North) 
        relative to origin (lat0, lon0).
        """
        R = 6371000 # Earth radius in meters
        
        phi = math.radians(lat)
        lmd = math.radians(lon)
        phi0 = math.radians(self.lat0)
        lmd0 = math.radians(self.lon0)
        
        dphi = phi - phi0
        dlmd = lmd - lmd0
        
        # x = Easting (Longitude difference), y = Northing (Latitude difference)
        x = R * math.cos(phi0) * dlmd
        y = R * dphi
    
        return x, y

    def gps_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn("Received GPS message with insufficient data")
            return

        lat = msg.data[0]
        lon = msg.data[1]

        # Use the set_origin function if it's not yet established
        if self.lat0 is None or self.lon0 is None:
            self.set_origin(lat, lon)

        # Convert to local XY
        x, y = self.latlon_to_xy_meters(lat, lon)

        # Create and publish the message
        xy_msg = Float32MultiArray()
        xy_msg.data = [x, y]
        self.publisher_.publish(xy_msg)

        self.get_logger().info(f"Local Position: X={x:.2f}m, Y={y:.2f}m")

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