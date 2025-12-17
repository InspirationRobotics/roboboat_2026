#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from RoboBoat_2026.API.GPS.gps_api import GPS     # <-- your existing GPS script
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

        self.get_logger().info("GPS node started.")

    def gps_callback(self, data):
        """Called automatically whenever the GPS thread gets new data."""
        if not data.is_valid():
            return

        msg = Float32MultiArray()
        msg.data = [float(data.lat), float(data.lon), float(data.heading)]
        self.pub.publish(msg)


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
