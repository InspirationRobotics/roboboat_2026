#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class JsonNavClient(Node):

    def __init__(self):
        super().__init__('json_nav_client')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.result_future = None

    def load_waypoints(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
        return data['waypoints'], data.get('frame_id', 'map')

    def send_goal_and_wait(self, wp, frame_id):
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(wp['x'])
        goal_msg.pose.pose.position.y = float(wp['y'])
        goal_msg.pose.pose.orientation.w = 1.0

        self.result_future = None

        # Send goal
        send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        # Spin until result_future is ready
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.result_future and self.result_future.done():
                return self.result_future.result().status

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.result_future = goal_handle.get_result_async()

    def feedback_callback(self, fb_msg):
        self.get_logger().info(
            f'Distance remaining: {fb_msg.feedback.distance_remaining:.2f}'
        )


def main():
    rclpy.init()
    node = JsonNavClient()

    node.get_logger().info('Waiting for action server...')
    node.client.wait_for_server()

    json_path = '/root/rb_ws/src/roboboat_2026/roboboat_2026/missions/waypoints/waypoint_001.json'
    waypoints, frame_id = node.load_waypoints(json_path)

    for i, wp in enumerate(waypoints):
        node.get_logger().info(f'\n=== Waypoint {i+1}/{len(waypoints)}: {wp["name"]} ===')

        status = node.send_goal_and_wait(wp, frame_id)

        if status != GoalStatus.STATUS_SUCCEEDED:
            node.get_logger().error('Waypoint failed, aborting mission')
            break

        node.get_logger().info('Waypoint reached')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
