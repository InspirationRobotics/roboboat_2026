#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from std_srvs.srv import Trigger
from roboboat_2026.api.servos.racquetball_launcher import ArdiunoCompound
from roboboat_2026.util import deviceHelper

class BallLauncherNode(Node):
    def __init__(self):
        super().__init__("ball_launcher_node")

        self.config = deviceHelper.variables
        self.port   = deviceHelper.dataFromConfig('ball_launcher')
        self.baurd_rate = self.config.get('ball_launcher').get('rate')
        
        # Initialize GPS (threaded=True)
        self.get_logger().info("Starting racquetball launcher...")
        self.launcher = ArdiunoCompound(port=self.port,baudrate=self.baurd_rate)
        
        self.srv = self.create_service(
            Trigger,
            'toggle_ball_launcher',
            self.launch_callback
        )  # do this in cli: ros2 service call /toggle_ball_launcher std_srvs/srv/Trigger "{}"

        self.srv_shutdown = self.create_service(
            Trigger,
            'release_ball_launcher',  # release the spring
            self.release_callback
        )  # do this in cli: ros2 service call /release_ball_launcher std_srvs/srv/Trigger "{}"
        self.get_logger().info("racquetball launcher node started.")

    def launch_callback(self, request, response):
        """Launch the ball, always do g first then a"""
        self.get_logger().info('Pump triggered')

        self.launcher.send_command('g')
        time.sleep(0.5)
        self.launcher.send_command('a')
        self.get_logger().info("Finished launching")

        response.success = True
        response.message = 'Pump triggered successfully'
        return response

    def release_callback(self, request, response):
        """Launch the ball, always do g first then a"""
        self.get_logger().warning('Releasing Spring on Racquetball launcher!')

        self.launcher.send_command('g')
        time.sleep(0.5)

        self.get_logger().info("Spring Released")
        
        response.success = True
        response.message = 'Spring released successfully'
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
