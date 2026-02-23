#
# !/usr/bin/env python3
import rclpy
import time
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
from threading import Thread, Lock
import json

from std_srvs.srv import Trigger
from roboboat_2026.api.util.gis_funcs import move_latlon



class FinalMission(Node):
    def __init__(self):
        super().__init__('jFinal')
        self.wp_feedback_sub = self.create_subscription(Bool, '/WP_finished', self.state_cb, 1)
        self.wp_pub = self.create_publisher(Float32MultiArray, '/nav2point', 10)
        self.wp_finished = False
        self.main_thread = Thread(target=self.run,daemon=True)
        self.pwm_pub = self.create_publisher(Float32MultiArray, 'teensy/pwm',10)

        # report pub
        self.report_pub = self.create_publisher(String, '/report',10)

        rclpy.spin(self)

    def load_waypoints(self, path):
        with open(path, 'r') as f:
            file = json.load(f)
            waypoints = file['waypoints']
            names = file['name']
            wp_ls = []
            for wp in waypoints:
                wp_ls.append([float(wp['lat']), float(wp['lon']),wp['task']])
        waypoint_dict = dict(zip(names, waypoints))
        return waypoint_dict

    def trigger_pump(self):
        request = Trigger.Request()
        self.future = self.water_pump.call_async(request)
        self.get_logger().info("Request Water Pump !")
        
    def trigger_launcher(self):
        request = Trigger.Request()
        # TODOs 
    def send_waypoint(self, point):
        path_msg = Float32MultiArray()
        path_msg.data = point
        self.wp_pub.publish(path_msg)

    def state_cb(self,msg):
        self.wp_finished = msg.data
    
    def report_wrap(self, msg):
        report_msg = String()
        report_msg.data = msg
        self.report_pub.publish(report_msg)

    def nav2point(self,point):
        """Point should be [lat,lon]"""
        self.wp_finished = False
        self.send_waypoint(point)
        while not self.wp_finished:
            time.sleep(1)
        self.get_logger().info(f"Reached wp {point}")
        
    def run(self):
        # Step 1, load all waypoints
        wpbook = self.load_waypoints("/root/rb_ws/src/roboboat_2026/roboboat_2026/missions/waypoints/waypoint_002.json")
        self.get_logger().info(f"Waypoints loaded")

        # Entry & Exit Gate mission
        for key,value in enumerate(wpbook):
            if str(key).startswith("E"):
                # This is Entry & Exit
                self.get_logger().info(f"Navigating to E {value}")
                self.nav2point(value)


        time.sleep(5)

        back_msg = Float32MultiArray()
        back_msg.data = [-0.6,0.0,0.0]
        self.pwm_pub.publish(back_msg)

        # self.trigger_pump()
def main():
    rclpy.init()
    client = FinalMission()

    client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
