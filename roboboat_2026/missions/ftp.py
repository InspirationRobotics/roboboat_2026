#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

import json
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory



class BoatNavigationClient(Node):

    def __init__(self):
        super().__init__('boat_navigation_client')
        
        # Create action client
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # Mission data
        self.waypoints = []
        self.mission_name = ""
        self.frame_id = "map"
        self.current_waypoint_index = 0
        
        self.get_logger().info('Boat navigation client initialized')

    def load_waypoints_from_json(self, json_file_path):
        """Load waypoints from JSON file"""
        try:
            with open(json_file_path, 'r') as f:
                data = json.load(f)
            
            self.mission_name = data.get('mission_name', 'Unnamed Mission')
            self.frame_id = data.get('frame_id', 'map')
            self.waypoints = data.get('waypoints', [])
            
            self.get_logger().info(f'Loaded mission: {self.mission_name}')
            self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')
            
            for wp in self.waypoints:
                self.get_logger().info(
                    f"  WP{wp['id']}: {wp['name']} at ({wp['x']}, {wp['y']})"
                )
            
            return True
            
        except FileNotFoundError:
            self.get_logger().error(f'Waypoint file not found: {json_file_path}')
            return False
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON format: {e}')
            return False
        except Exception as e:
            self.get_logger().error(f'Error loading waypoints: {e}')
            return False

    def send_waypoint(self, waypoint_data):
        """Send waypoint goal to the boat"""
        
        # Wait for the action server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available!')
            return None
        
        # Extract waypoint data
        x = waypoint_data['x']
        y = waypoint_data['y']
        name = waypoint_data.get('name', f"WP{waypoint_data.get('id', '?')}")
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set target position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Set orientation
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(
            f'Sending waypoint: {name} at ({x}, {y})'
        )
        
        # Send goal with feedback callback
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        send_goal_future.add_done_callback(self.goal_response_callback)
        return send_goal_future

    def goal_response_callback(self, future):
        """Called when server accepts or rejects the goal"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warning('Waypoint goal rejected by server')
            return
        
        self.get_logger().info('Waypoint goal accepted by server')
        
        # Store goal handle
        self.current_goal_handle = goal_handle
        
        # Get result when action completes
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Called when feedback is received"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        
        current_x = feedback.current_pose.pose.position.x
        current_y = feedback.current_pose.pose.position.y
        
        self.get_logger().info(
            f'📏 Distance: {distance:.2f}m | '
            f'Position: ({current_x:.2f}, {current_y:.2f})'
        )

    def result_callback(self, future):
        """Called when the action completes"""
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Waypoint reached successfully!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warning('Navigation aborted')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning('Navigation canceled')
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')

    def execute_mission(self):
        """Execute all waypoints in the mission sequentially"""
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded!')
            return False
        
        self.get_logger().info(
            f'Starting mission: {self.mission_name}'
        )
        self.get_logger().info(
            f'Total waypoints: {len(self.waypoints)}'
        )
        
        for i, waypoint in enumerate(self.waypoints):
            self.current_waypoint_index = i
            
            self.get_logger().info(
                f'\n=== Waypoint {i+1}/{len(self.waypoints)}: '
                f'{waypoint["name"]} ==='
            )
            
            if 'description' in waypoint:
                self.get_logger().info(f'Description: {waypoint["description"]}')
            
            # Send waypoint
            future = self.send_waypoint(waypoint)
            
            if future is None:
                self.get_logger().error('Failed to send waypoint, aborting mission')
                return False
            
            # Wait for waypoint to complete
            rclpy.spin_until_future_complete(self, future)
            
            # Brief pause between waypoints
            import time
            time.sleep(1.0)
        
        self.get_logger().info('Mission complete!')
        return True

    def cancel_mission(self):
        """Cancel the current navigation goal"""
        if hasattr(self, 'current_goal_handle'):
            self.get_logger().info('Canceling mission...')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_callback)
        else:
            self.get_logger().warning('No active goal to cancel')

    def cancel_callback(self, future):
        """Called when goal cancellation completes"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().warning('Goal failed to cancel')


def main(args=None):
    rclpy.init(args=args)
    
    client = BoatNavigationClient()
    
    # Get the package share directory
    package_share_dir = "/root/rb_ws/src/roboboat_2026/roboboat_2026"
    waypoint_file = os.path.join(
            package_share_dir,
            'missions',
            'waypoints',
            'waypoint001.json'
        )
    
    # Load waypoints
    if not client.load_waypoints_from_json(waypoint_file):
        client.get_logger().error('Failed to load waypoints, exiting')
        rclpy.shutdown()
        return
    
    # Execute mission
    try:
        client.execute_mission()
    except KeyboardInterrupt:
        client.get_logger().info('\nMission interrupted by user')
        client.cancel_mission()
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()