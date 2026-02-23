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

        # WP related
        self.wp_feedback_sub = self.create_subscription(Bool, '/WP_finished', self.state_cb, 1)
        self.wp_pub = self.create_publisher(Float32MultiArray, '/nav2point', 10)
        self.wp_finished = False

        # Control related
        self.main_thread = Thread(target=self.run,daemon=True)
        self.main_thread.start()
        self.pwm_pub = self.create_publisher(Float32MultiArray, 'teensy/pwm',10)

        # report pub
        self.report_pub = self.create_publisher(String, '/report',10)

        # GPS Sub
        self.gps_sub = self.create_subscription(Float32MultiArray, '/GPS', self.gps_cb, 1)
        self.gps_pos = None

        rclpy.spin(self)

    def load_waypoints(self, path):
        with open(path, 'r') as f:
            file = json.load(f)
            print(file)
            waypoints = file['waypoints']
            wp_ls = []
            waypoint_dict = {}
            # print(waypoints)
            for wp in waypoints:
                waypoint_dict[wp['name']] = [float(wp['lat']), float(wp['lon']),wp['task']]
            # waypoint_dict = dict(zip(name_ls, wp_ls))
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
    
    def gps_cb(self,msg):
        self.gps_pos = [msg.data[0],msg.data[1]]
        
    def report_wrap(self, msg):
        report_msg = String()
        report_msg.data = msg
        self.report_pub.publish(report_msg)

    def nav2point(self,point):
        """Point should be [lat,lon]"""
        self.get_logger().info(f"Navigate to {point}")
        self.wp_finished = False
        self.send_waypoint(point)
        while not self.wp_finished:
            time.sleep(1)
        self.get_logger().info(f"Reached wp {point}")
        
    def run(self):
        print("Start running")
        # Step 1, load all waypoints
        wpbook = self.load_waypoints("/root/rb_ws/src/roboboat_2026/roboboat_2026/missions/waypoints/waypoint_001.json")
        self.get_logger().info(f"Waypoints loaded")
        self.get_logger().info(f"{wpbook}")
        
        while self.gps_pos is None:
            self.get_logger().info("Waiting for GPS info")
            time.sleep(1)

        # Entry & Exit Gate mission
        self.get_logger().info("Starting Entry Exit Gate")
        self.report_wrap(f"GatePass,ENTRY,{self.gps_pos[0]},{self.gps_pos[1]}")
        for key,value in wpbook.items():
            print(key)
            print(str(key).startswith("E"))
            if str(key).startswith("E"):
                self.get_logger().info(f"Navigating to E {value}")
                self.nav2point(value)
        self.report_wrap(f"GatePass,EXIT,{self.gps_pos[0]},{self.gps_pos[1]}")

        # Nav Channel
        for key,value in enumerate(wpbook):
            if str(key).startswith("N"):
                self.get_logger().info(f"Navigating to N {value}")
                self.nav2point(value)
            if key=="N2":
                # Report some stuff
                self.report_wrap(f"ObjectDetected,BOAT,RED,32.112345,-21.12345,1,NAV_CHANNEL")
                self.report_wrap(f"ObjectDetected,BUOY,RED,32.112345,-21.12345,2,NAV_CHANNEL")
                self.report_wrap(f"ObjectDetected,LIGHT_BEACON,RED,32.112345,-21.12345,3,NAV_CHANNEL")

        # Docking
        for key,value in enumerate(wpbook):
            if str(key).startswith("D"):
                self.get_logger().info(f"Navigating to D {value}")
                self.nav2point(value)

            if key=="D3":
                time.sleep(5)
                back_msg = Float32MultiArray()
                back_msg.data = [-0.6,0.0,0.0]
                self.pwm_pub.publish(back_msg)

        time.sleep(5)
        
        back_msg = Float32MultiArray()
        back_msg.data = [-0.6,0.0,0.0]
        self.pwm_pub.publish(back_msg)

        # Speed Challenge
        for key,value in enumerate(wpbook):
            if str(key).startswith("R"):
                self.get_logger().info(f"Navigating to R {value}")
                self.nav2point(value)

        # Return home
        for key,value in enumerate(wpbook):
            if str(key).startswith("R"):
                self.get_logger().info(f"Navigating to R {value}")
                self.nav2point(value)

    def destroy_node(self):
        self.get_logger().info("Custom destroy_node")
        self.timer.cancel()
        
        super().destroy_node()

def main():
    rclpy.init()
    client = FinalMission()
    self.main_thread.join()
    client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
