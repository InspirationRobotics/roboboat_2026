#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from roboboat_2026.api.gps.gps_api import GPS     # <-- your existing GPS script
import threading


class GPSNode(Node):
    def __init__(self):
        super().__init__("gps_node")

        # Publisher
        self.pub = self.create_publisher(Float32MultiArray, "/GPS", 10)

        # Initialize GPS (threaded=True)
        self.get_logger().info("Starting GPS...")
        self.gps = GPS(serialport="/dev/ttyUSB0",
                       baudrate=115200,
                       callback=self.gps_callback,   # GPS will call this when data updates
                       threaded=True)

        self.srv_calibrate= self.create_service(
            Trigger,
            'calibrate_gps',  # set current heading to 0
            self.calibrate_callback
        )  # do this in cli: ros2 service call /calibrate_gps std_srvs/srv/Trigger "{}"
        self.get_logger().info("GPS node started.")
    def gps_callback(self, data):
        """Called automatically whenever the GPS thread gets new data."""
        if not data.is_valid():
            return

        msg = Float32MultiArray()
        msg.data = [float(data.lat), float(data.lon), float(data.heading)]
        self.pub.publish(msg)
    def calibrate_callback(self, request, response):
        """Launch the ball, always do g first then a"""
        self.get_logger().info('calibrating GPS!')

        self.gps.calibrate_heading_offset(calib_time=5)
        response.success = True
        response.message = 'GPS calibrated'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
