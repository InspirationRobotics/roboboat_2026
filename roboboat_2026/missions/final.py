#!/usr/bin/env python3
import rclpy
import time
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
from threading import Thread, Lock
import json
from std_srvs.srv import Trigger

class FinalMission(Node):
    def __init__(self):
        super().__init__('jFinal')
        self.wp_path_pub = self.create_publisher(String, '/waypoint_path', 10)
        self.wp_feedback_sub = self.create_subscription(Bool, '/WP_finished', self.state_cb, 1)
        self.wp_finished = False
        self.main_thread = Thread(target=self.run,daemon=True)
        self.pwm_pub = self.create_publisher(Float32MultiArray, 'teensy/pwm',10)
        self.get_logger().info("Mission ready")
        self.water_pump = self.create_client(Trigger, 'toggle_water_pump')
        self.launcher = self.create_client(Trigger, 'toggle_ball_launcher')
        while not self.water_pump.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.pump_req = Trigger.Request()

        # while not self.launcher.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("Waiting for launcher")
        self.launcher_req = Trigger.Request()
        self.main_thread.start()


#         rclpy.spin(self)

        # report pub
        self.report_pub = self.create_publisher(String, '/report',10)
        rclpy.spin(self)

    def trigger_pump(self):
        request = Trigger.Request()
        self.future = self.water_pump.call_async(request)
        self.get_logger().info("Request Water Pump !")
        
    def trigger_launcher(self):
        request = Trigger.Request()
        # TODOs 
    def send_waypoints(self, path):
        path_msg = String()
        path_msg.data = path
        self.wp_path_pub.publish(path_msg)

    def state_cb(self,msg):
        self.wp_finished = msg.data
    
    def report_wrap(self, msg):
        report_msg = String()
        report_msg.data = msg
        self.report_pub.publish(report_msg)

    def run(self):
        
        # Entry & Exit Gate mission
        # self.report_wrap("GatePass,ENTRY,32.112345,-21.12345")
        self.get_logger().info("Waiting for 30s to do harbor alert")
        time.sleep(30)
        self.wp_finished = False
        self.send_waypoints(path="/root/rb_ws/src/roboboat_2026/roboboat_2026/missions/waypoints/waypoint_001.json")
        # self.report_wrap("GatePass,EXIT,32.112345,-21.12345")

        while not self.wp_finished:
           # print("I;m sleeping")
            time.sleep(1)

        # Nav Channel
        # self.wp_finished = False
        # self.send_waypoints(path="/root/rb_ws/src/roboboat_2026/roboboat_2026/missions/waypoints/waypoint_002.json")
        # while not self.wp_finished:
        #     time.sleep(1)
        
        # Do Water pummp
        # self.trigger_pump()

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
