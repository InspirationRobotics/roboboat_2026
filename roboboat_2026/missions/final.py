#!/usr/bin/env python3
import rclpy
import time
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
from threading import Thread, Lock
import json

class FinalMission(Node):
    def __init__(self):
        super().__init__('jFinal')
        self.wp_path_pub = self.create_publisher(String, '/waypoint_path', 10)
        self.wp_feedback_sub = self.create_subscription(Bool, '/WP_finished', self.state_cb, 1)
        self.wp_finished = False
        self.main_thread = Thread(target=self.run,daemon=True)
        self.main_thread.start()
        self.get_logger().info("Mission ready")
        rclpy.spin(self)


    def send_waypoints(self, path):
        path_msg = String()
        path_msg.data = path
        self.wp_path_pub.publish(path_msg)

    def state_cb(self,msg):
        self.wp_finished = msg.data
    
    def run(self):
        # Entry & Exit Gate mission
        self.wp_finished = False
        self.send_waypoints(path="/root/rb_ws/src/roboboat_2026/roboboat_2026/missions/waypoints/waypoint_001.json")

        while not self.wp_finished:
            print("I;m sleeping")
            time.sleep(1)

        # Nav Channel
        self.wp_finished = False
        self.send_waypoints(path="/root/rb_ws/src/roboboat_2026/roboboat_2026/missions/waypoints/waypoint_001.json")
        while not self.wp_finished:
            time.sleep(1)

def main():
    rclpy.init()
    client = FinalMission()

    client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
