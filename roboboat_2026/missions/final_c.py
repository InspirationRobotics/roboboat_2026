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
        print(point)
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
        tasks = ['UNKNOWN','NONE','ENTRY_EXIT','NAV_CHANNEL','SPEED_CHALLENGE','OBJECT_DELIVERY','DOCKING','SOUND_SIGNAL']
        print(tasks.index(point[2]))
        print(point)
        self.send_waypoint([point[0],point[1],float(tasks.index(point[2]))])
        while not self.wp_finished:
            time.sleep(1)
        self.get_logger().info(f"Reached wp {point}")
    
    def move_by_time(self,pwms,t):
        msg = Float32MultiArray()
        msg.data = pwms
        self.pwm_pub.publish(msg)
        time.sleep(t)
        msg.data = [0.0,0.0,0.0]
        self.pwm_pub.publish(msg)

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
        return_home_pos = self.gps_pos
        self.get_logger().info(f"Return home pos is {return_home_pos}")
        self.report_wrap(f"GatePass,ENTRY,{self.gps_pos[0]},{self.gps_pos[1]}")
        new_lat, new_lon = move_latlon(self.gps_pos[0],self.gps[1],direction='east',distance_m=20)
        self.nav2point([new_lat,new_lon])
        self.report_wrap(f"GatePass,EXIT,{self.gps_pos[0]},{self.gps_pos[1]}")

        # Nav Channel
        self.nav2point(wpbook['N1'])
        self.report_wrap(f"ObjectDetected,BOAT,RED,32.112345,-21.12345,1,NAV_CHANNEL")
        self.nav2point(wpbook['N2'])
        self.nav2point(wpbook['N3'])
        self.report_wrap(f"ObjectDetected,BUOY,RED,32.112345,-21.12345,2,NAV_CHANNEL")
        self.report_wrap(f"ObjectDetected,LIGHT_BEACON,RED,32.112345,-21.12345,3,NAV_CHANNEL")

        # Docking
        self.nav2point(wpbook['D1'])
        self.nav2point(wpbook['D2'])
        self.nav2point(wpbook['D3'])

        time.sleep(5)
        
        self.move_by_time(pwms=[-0.6,0.0,0.0],t=4)

        # Speed Challenge
        for key,value in wpbook.items():
            if str(key).startswith("S"):
                self.get_logger().info(f"Navigating to S {value}")
                self.nav2point(value)

        # Return home
        for key,value in wpbook.items():
            if str(key).startswith("R"):
                self.get_logger().info(f"Navigating to R {value}")
                self.nav2point(value)

    def destroy_node(self):
        self.get_logger().info("Custom destroy_node")
        self.timer.cancel()
        self.main_thread.join()    
        super().destroy_node()

def main():
    rclpy.init()
    client = FinalMission()
    client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
