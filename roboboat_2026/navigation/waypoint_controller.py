#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from nav2_msgs.action import NavigateToPose
from roboboat_2026.util.helper import heading_error, get_heading_from_coords

def simpleControl(distance, heading_error):
    max_surge = 0.5
    max_yaw = 0.8
    res = [max_surge,0.0]
    if distance < 4:
        res[0] = float(distance/4) * max_surge
    
    res[1] = (heading_error/180) * max_yaw

    return res[0], res[1]


class WaypointNav(Node):

    def __init__(self):
        super().__init__('waypoint_nav_node')

        # State
        self.position = None
        self.heading = None

        # Subscribers
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

        # Action Server
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )

        self.get_logger().info('NavigateToPose action server started')

    def pose_callback(self, msg):
        self.position = [
            msg.pose.position.x,
            msg.pose.position.y
        ]

        self.get_logger().info(f"current pose: {self.position}")
 

    def gps_callback(self, msg):
        self.heading = msg.data[2]

    def execute_callback(self, goal_handle):
        goal_pose = goal_handle.request.pose.pose.position
        x_goal = goal_pose.x
        y_goal = goal_pose.y

        self.get_logger().info(
            f'NavigateToPose goal received: x={x_goal}, y={y_goal}'
        )

        feedback = NavigateToPose.Feedback()
        result = NavigateToPose.Result()

        tolerance = 0.8  # meters

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.position is None:
                time.sleep(1)
                continue

            # find distance and heading error
            self.get_logger().info(f"current_position: {self.position}")
            dx = x_goal - self.position[0]
            dy = y_goal - self.position[1]
            distance = math.hypot(dx, dy)
            self.get_logger().info(f"{distance} away from goal")
            desire_heading = get_heading_from_coords(dx,dy)
            error_heading = heading_error(self.heading, desire_heading)


            # Feedback
            feedback.distance_remaining = float(distance)
            goal_handle.publish_feedback(feedback)

            # Optional: publish current pose feedback
            feedback.current_pose = PoseStamped()
            feedback.current_pose.pose.position.x = self.position[0]
            feedback.current_pose.pose.position.y = self.position[1]

            if distance < tolerance:
                self.get_logger().info('Goal reached')
                result.result = 0  # SUCCESS
                goal_handle.succeed()
                return result

            # Example PWM output
            pwm = Float32MultiArray()
            surge, yaw = simpleControl(distance,error_heading)
            pwm.data = [surge,0.0,yaw]
            self.pwm_pub.publish(pwm)

            time.sleep(0.1)

        result.result = 1  # FAILURE
        goal_handle.abort()
        return result


def main():
    rclpy.init()
    node = WaypointNav()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
