#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import sys
import time


def main():

    
    waypoint_path = "/root/rb_ws/src/roboboat_2026/roboboat_2026/missions/waypoints/waypoint_001.json"
    
    rclpy.init()
    node = Node('waypoint_client')
    
    # Create publisher for path
    path_pub = node.create_publisher(String, '/waypoint_path', 10)
    state_pub = node.create_publisher(Bool, '/WPFollower_state', 10)
    # Create service client
    client = node.create_client(Trigger, 'follow_waypoints')
    
    # Wait for service
    node.get_logger().info('Waiting for follow_waypoints service...')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting...')
    
    # Publish waypoint path
    msg = String()
    msg.data = waypoint_path
    path_pub.publish(msg)
    node.get_logger().info(f'Published waypoint path: {waypoint_path}')
    
    state_msg = Bool()
    state_msg.data = True
    state_pub.publish(state_msg)
    node.get_logger().info(f'Set state to Active')
    # Wait a bit for message to be received
    time.sleep(0.5)
    
    response = future.result()
    if response.success:
        node.get_logger().info(f'✓ SUCCESS: {response.message}')
    else:
        node.get_logger().error(f'✗ FAILED: {response.message}')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()