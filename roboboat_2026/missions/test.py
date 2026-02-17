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
        self.navchannel_activate_pub = self.create_publisher(Bool, '/NavChannel_start', 10)
        self.wp_feedback_sub = self.create_subscription(Bool, '/WP_finished', self.waypoint_cb, 1)
        self.nc_feedback_sub_feedback_sub = self.create_subscription(Bool, '/NavChannel_finished', self.navchannel_cb, 1)
        self.wp_finished = False
        self.nav_channel_finished = False
        self.main_thread = Thread(target=self.run,daemon=True)
        self.main_thread.start()
        self.get_logger().info("Mission ready")
        rclpy.spin(self)


    def send_waypoints(self, path):
        path_msg = String()
        path_msg.data = path
        self.wp_path_pub.publish(path_msg)

    def activate_nav_channel(self):
        msg = Bool()
        msg.data = True
        self.navchannel_activate_pub.publish(msg)
    def waypoint_cb(self,msg):
        self.wp_finished = msg.data
    
    def navchannel_cb(self,msg):
        self.nav_channel_finished = msg.data

    def run(self):
        self.activate_nav_channel()

        while not self.nav_channel_finished:
            print("waiting for nav channel mission")
            time.sleep(1)
        
        

def main():
    rclpy.init()
    client = FinalMission()

    client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
