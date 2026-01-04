#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import json

class JsonWaypointClient(Node):
    def __init__(self):
        super().__init__('json_waypoint_client')
        self.pub = self.create_publisher(Float32MultiArray, '/add_waypoint', 10)
        self.get_logger().info("Client ready")

    def load_waypoints(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
        return data['waypoints']

    def send_waypoints(self, waypoints):
        for wp in waypoints:
            msg = Float32MultiArray()
            msg.data = [float(wp['x']), float(wp['y'])]
            self.pub.publish(msg)
            self.get_logger().info(f"Sent waypoint: {msg.data}")

def main():
    rclpy.init()
    client = JsonWaypointClient()

    json_path = "/root/rb_ws/src/roboboat_2026/roboboat_2026/missions/waypoints/waypoint_001.json"
    waypoints = client.load_waypoints(json_path)
    client.send_waypoints(waypoints)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
