#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Empty

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from nav2_msgs.action import NavigateToPose
from roboboat_2026.util.helper import heading_error, get_heading_from_coords

def quaternion_to_yaw(q):
    """
    q: geometry_msgs.msg.Quaternion
    returns yaw in degrees
    """
    return math.degrees(math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    ))

def simpleControl(distance, heading_error):
    max_surge = 0.6
    max_yaw = 0.8
    res = [max_surge,0.0]
    if distance < 5:
        res[0] = float(distance/4) * max_surge
    
    res[1] = min((heading_error/180) * max_yaw,0.3) if heading_error > 10 else 0.0

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
        # control loop
        self.active_goal = None
        self.goal_handle = None

        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info('NavigateToPose action server started')

    def pose_callback(self, msg):
        self.position = [
            msg.pose.position.x,
            msg.pose.position.y
        ]

        # self.heading = quaternion_to_yaw(msg.pose.orientation)
        # self.get_logger().info(f"current pose: {self.position}")
 

    def gps_callback(self, msg):
        self.heading = msg.data[2]

    def control_loop(self):

        if self.active_goal is None:
            return

        if self.position is None or self.heading is None:
            return

        dx = self.active_goal.x - self.position[0]
        dy = self.active_goal.y - self.position[1]
        distance = math.hypot(dx, dy)

        self.get_logger().info(
            f"{distance:.2f} m | heading: {self.heading:.1f}"
        )

        desire_heading = get_heading_from_coords(dx, dy)
        error_heading = heading_error(self.heading, desire_heading)

        feedback = NavigateToPose.Feedback()
        feedback.distance_remaining = float(distance)
        self.goal_handle.publish_feedback(feedback)

        tolerance = 0.8
        if distance < tolerance:
            self.get_logger().info('Goal reached')
            if self.goal_handle.is_active:
                self.goal_handle.succeed()

            self.active_goal = None
            self.goal_handle = None
            return

        surge, yaw = simpleControl(distance, error_heading)

        pwm = Float32MultiArray()
        pwm.data = [float(surge), 0.0, float(yaw)]
        self.pwm_pub.publish(pwm)

    def execute_callback(self, goal_handle):
        self.goal_handle = goal_handle
        self.active_goal = goal_handle.request.pose.pose.position

        self.get_logger().info(
            f'NavigateToPose goal received: x={self.active_goal.x}, y={self.active_goal.y}'
        )

        return NavigateToPose.Result()  # result returned later




def main():
    rclpy.init()
    node = WaypointNav()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
