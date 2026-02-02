#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Empty

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, String
from roboboat_2026.util.helper import heading_error, get_heading_from_coords

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

        step_up = abs(res[0] - self.last_surge)
        if step_up > 0.1:
            # force a small increase
            if res[0] > self.last_surge:
                res[0] = self.last_surge + 0.1
            else:
                res[0] = self.last_surge - 0.1

        res[0] = min(self.max_surge,res[0]) 
        return res[0], res[1]
    

def simpleControl(distance, heading_error):
    max_surge = 0.8
    max_yaw = 0.5
    res = [max_surge,0.0]
    if distance < 5:
        res[0] = max(float(distance/4) * max_surge,0.2)
    
    if heading_error>0:
        res[1] = max(abs(heading_error/180) * max_yaw,0.3) if abs(heading_error) > 10 else 0.0
    else:
        res[1] = - max(abs(heading_error/180) * max_yaw,0.3) if abs(heading_error) > 10 else 0.0

    return res[0], res[1]

class WaypointService(Node):
    def __init__(self):
        super().__init__('waypoint_service')
        self.queue = []
         # State
        self.position = None
        self.heading = None
        self.current_task = None
        self.controller = SimpleControl()

        self.task_map = ['UNKNOWN','NONE','NAV_CHANNEL','SPEED_CHALLENGE','OBJECT_DELIVERY','DOCKING','SOUND_SIGNAL']

        # Subscribers
        self.srv_sub = self.create_subscription(
            Float32MultiArray,
            '/add_waypoint',
            self.add_waypoint_callback,
            10
        )

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

        # Control loop
        self.timer = self.create_timer(0.05, self.control_loop) # 20 Hz
        self.active_goal = None

        self.get_logger().info("Waypoint server ready")

    def pose_callback(self, msg):
        self.position = [
            msg.pose.position.x,
            msg.pose.position.y
        ]

        # self.heading = quaternion_to_yaw(msg.pose.orientation)
        # self.get_logger().info(f"current pose: {self.position}")
 

    def gps_callback(self, msg):
        self.heading = msg.data[2]

    def add_waypoint_callback(self, msg: Float32MultiArray):
        self.get_logger().info("received waypoints")
        if len(msg.data) != 3:
            self.get_logger().error("Waypoint must be [x, y, task]")
            return
        self.queue.append(msg.data)
        self.get_logger().info(f"Waypoint added to queue: {msg.data}")

    def control_loop(self):
        if self.position is None or self.heading is None:
            return

        if self.active_goal is None and self.queue:
            self.active_goal = self.queue.pop(0)
            self.get_logger().info(f"Starting waypoint: {self.active_goal}")

        if self.active_goal is None:
            return
        
            
        x, y, task = self.active_goal
        if self.current_task != task:
            msg = String()
            msg.data = self.task_map[int(task)]
            self.current_task = int(task)
            self.task_pub.publish(msg)
        dx = x - self.position[0]
        dy = y - self.position[1]
        distance = math.hypot(dx, dy)


        desire_heading = get_heading_from_coords(dx, dy)
        error_heading = heading_error(self.heading, desire_heading)

        # Goal reached
        if distance < 0.5:
            self.get_logger().info(f"Reached waypoint x={x}, y={y}")
            if self.queue:
                self.active_goal = self.queue.pop(0)
            else:
                self.active_goal = None
                pwm = Float32MultiArray()
                pwm.data = [0.0, 0.0, 0.0]
                self.pwm_pub.publish(pwm)
            return

        self.get_logger().info(f"{distance:.2f} m to goal | heading error: {error_heading:.1f}")
        surge, yaw = self.controller.control(distance, error_heading)
        pwm = Float32MultiArray()
        pwm.data = [float(surge), 0.0, float(yaw)]
        self.pwm_pub.publish(pwm)

def main():
    rclpy.init()
    node = WaypointService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
