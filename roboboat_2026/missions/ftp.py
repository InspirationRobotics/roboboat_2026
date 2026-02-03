#!/usr/bin/env python3
import rclpy
import math
import time
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
from roboboat_2026.util.helper import heading_error, get_heading_from_coords
import json

class SimpleControl:
    def __init__(self):
        self.max_surge = 1
        self.max_yaw = 0.6
        self.last_surge = 0.0
    
    def control(self,distance, heading_error):

        res = [self.last_surge,0.0]        
        if distance < 5:
            res[0] = max(float(distance/4) * self.max_surge,0.2)
        
        if heading_error>0:
            res[1] = max(abs(heading_error/180) * self.max_yaw,0.3) if abs(heading_error) > 10 else 0.0
        else:
            res[1] = - max(abs(heading_error/180) * self.max_yaw,0.3) if abs(heading_error) > 10 else 0.0

        step_up = res[0] - self.last_surge
        if step_up > 0.05:
            # force a small increase
            res[0] = self.last_surge + 0.05


        res[0] = min(self.max_surge,res[0]) 
        return res[0], res[1]

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('json_waypoint_client')
        self.pub = self.create_publisher(Float32MultiArray, '/add_waypoint', 10)
        self.get_logger().info("Waypoint Follower ready")
        self.waypoints:list = None
        self.desire_x:float = None
        self.desire_y:float = None
        self.cur_task:str = None

        self.controller = SimpleControl()

        self.heading:float = None
        self.position:list = None
        self.pwms = [0.0,0.0,0.0]

        # Ros 2
        self.create_subscription(
            PoseStamped,
            '/fused/pose',
            self.pose_callback,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            '/GPS',
            self.gps_callback,
            10
        )

        # Publisher to Teensy
        self.pwm_pub = self.create_publisher(
            Float32MultiArray,
            '/teensy/pwm',
            10
        )

        self.task_pub = self.create_publisher(
            String,
            '/cur_task',
            10
        )

    def load_waypoints(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
        return data['waypoints']

    def pose_callback(self, msg):
        self.position = [
            msg.pose.position.x,
            msg.pose.position.y
        ]
    
    def gps_callback(self, msg):
        self.heading = msg.data[2]

    def pub_task(self):
        if self.cur_task is not None:
            msg = String()
            msg.data = self.cur_task
            self.task_pub.publish(msg)

    def pub_teensy(self,pwms):
        msg = Float32MultiArray()
        msg.data = pwms
        self.pwm_pub.publish(msg)

    def run(self, path):
        self.waypoints = self.load_waypoints(path)
        reach_goal = False
        for wp in self.waypoints:
            self.desire_x = float(wp['x'])
            self.desire_y = float(wp['y'])
            self.cur_task = wp['task']
            while not reach_goal:
                if self.cur_task=="object_delivery":
                    # when we want to shoot water
                    pass
            
                if self.position is None or self.heading is None:
                    # check if the localization is running
                    continue
                
                self.pub_task()
                

                dx = self.desire_x - self.position[0]
                dy = self.desire_y - self.position[1]
                distance = math.hypot(dx, dy)

                desire_heading = get_heading_from_coords(dx, dy)
                error_heading = heading_error(self.heading, desire_heading)

                # Goal reached
                if distance < 0.5:
                    self.get_logger().info(f"Reached waypoint x={self.desire_x}, y={self.desire_y}")
                    reach_goal = True
                    pwm = Float32MultiArray()
                    pwm.data = [0.0, 0.0, 0.0]
                    self.pwm_pub.publish(pwm)
                else:
                    self.get_logger().info(f"{distance:.2f} m to goal | heading error: {error_heading:.1f}")
                    surge, yaw = self.controller.control(distance, error_heading)
                    self.pub_teensy([surge,0.0,yaw])
                
                time.sleep(0.1) # ~ 10 Hz loop


def main():
    rclpy.init()
    client = WaypointFollower()

    json_path = "/root/rb_ws/src/roboboat_2026/roboboat_2026/missions/waypoints/waypoint_001.json"
    # "/root/rb_ws/src/roboboat_2026/roboboat_2026/missions/waypoints/waypoint_001.json"  
    # "/home/chaser0721/rb_ws/src/roboboat_2026/roboboat_2026/missions/waypoints/waypoint_001.json"
    client.run(path=json_path)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
