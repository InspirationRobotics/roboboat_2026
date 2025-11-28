#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix, NavSatStatus
from threading import Thread, Lock
import serial
import time


class Teensy:
    def __init__(self, port="/dev/ttyACM1", baudrate=115200):
        self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)

        self.send_PWM([0,0,0])
        time.sleep(1)

    def send_PWM(self, command):
        parsed = ",".join(str(x) for x in command) + "\n"
        self.arduino.write(parsed.encode())

# ==========================================================
#  ROS 2 Node
# ==========================================================

class ThrusterControllerNode(Node):
    def __init__(self):
        super().__init__('thruster_controller')

        self.logger = self.get_logger()
        self.logger.info("Thruster Controller Node Started")

        # Thruster driver
        self.teensy = Teensy()

        # Subscribe to normalized surge/sway/yaw command
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/teensy/pwm',
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(
            NavSatFix,
            '/teensy/GPS',
            10
        )

    def listener_callback(self, msg):
        """
        Expect msg.data = [surge, sway, yaw], each in [-1, 1]
        """
        if len(msg.data) != 3:
            self.get_logger().error("cmd_vel_normalized must be [surge, sway, yaw]")
            return

        surge = round(float(msg.data[0]), 6)
        sway  = round(float(msg.data[1]), 6)
        yaw   = round(float(msg.data[2]), 6)

        self.logger.info(f"pwm: {[surge,sway,yaw]}")
        self.teensy.send_PWM([surge,sway,yaw])

    def destroy_node(self):
        self.get_logger().info("Stopping thrusters...")
        self.t200.stop_thrusters()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C received")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
