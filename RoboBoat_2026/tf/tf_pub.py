#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from builtin_interfaces.msg import Time

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher_node')
        
        # Create the broadcaster
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Create the broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Publishing rate
        self.timer = self.create_timer(0.1, self.publish_tf)  # 10 Hz

        self.get_logger().info('TF published')

    def pub_oakd(self):
        t = TransformStamped()

        # Time is ignored for tf_static, but must be filled
        t.header.stamp = self.get_clock().now().to_msg()

        # Parent → Child
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'oak-d-base-frame'

        # Translation
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Identity rotation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Publish 
        self.broadcaster.sendTransform(t)

        

    def pub_livox(self):
        t = TransformStamped()

        # Time is ignored for tf_static, but must be filled
        t.header.stamp = self.get_clock().now().to_msg()

        # Parent → Child
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'livox_frame'

        # Translation
        t.transform.translation.x = 0.0
        t.transform.translation.y = 1.0
        t.transform.translation.z = 0.0

        # Identity rotation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Publish
        self.broadcaster.sendTransform(t)

        
    def publish_tf(self):
        self.pub_oakd()
        self.pub_livox()

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
