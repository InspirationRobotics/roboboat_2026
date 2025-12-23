#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from std_srvs.srv import Trigger
from roboboat_2026.api.servos.racquetball_launcher import ArdiunoCompound

class BallLauncherNode(Node):
    def __init__(self):
        super().__init__("ball_launcher_node")


        # Initialize GPS (threaded=True)
        self.get_logger().info("Starting racquetball launcher...")
        self.launcher = ArdiunoCompound(port="/dev/ttyACM1")
        
        self.srv = self.create_service(
            Trigger,
            'toggle_ball_launcher',
            self.launch_callback
        )
        self.get_logger().info("GPS node started.")

    def launch_callback(self, request, response):
        """Launch the ball, always do g first then a"""
        self.get_logger().info('Pump triggered')

        self.launcher.send_command('g')
        time.sleep(0.5)
        self.launcher.send_command('a')
        response.success = True
        response.message = 'Pump triggered successfully'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BallLauncherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
