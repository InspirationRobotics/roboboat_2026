import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool, Int32
from roboboat_2026.util import deviceHelper
from threading import Thread, Lock


class ArduinoNano:
    def __init__(self, port, baudrate=9600):
        """
        Initializes the serial connection to the Mini Maestro Servo Controller.

        Args:
            - port (str): The COM port (e.g., "COM3" for Windows or "/dev/ttyUSB0" for Linux/Mac).
            - baudrate (int): Communication speed (default is 9600).
        """
        self.serial_conn = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Allow time for the connection to establish

    def send_command(self, msg):
        """
        Sends int / float / str / bytes over serial.
        Appends newline for Arduino compatibility.
        """
        if isinstance(msg, bytes):
            data = msg
        else:
            data = f"{msg}\n".encode("ascii")

        self.serial_conn.write(data)
        self.serial_conn.flush()

    def close(self):
        """Closes the serial connection."""
        if self.serial_conn.is_open:
            self.serial_conn.close()


class LEDNode(Node):
    def __init__(self):
        super().__init__('LED')

        self.config = deviceHelper.variables
        self.port   = deviceHelper.dataFromConfig('led')
        self.baurd_rate = self.config.get('led').get('rate')

        # Arduino Nano stuff
        self.LED = ArduinoNano(port=self.port,baudrate=self.baurd_rate)
        self.mode = "AUTO"
        self.lock = Lock()

        # Subscribe to led_state topic
        self.request_sub= self.create_subscription(
            Int32,
            '/led_state',
            self.request_callback,
            10
        )   # to send in cli: ros2 topic pub /led_state std_msgs/msg/Int32 "{data: 1}" --once

        # see if the robot is in manual or autonomous
        self.state_sub = self.create_subscription(
            String,
            '/robot_state',
            self.state_callback,
            10
        )   

        self.get_logger().info("LED Node Started")
                
    def request_callback(self, msg):
        """
        Expect msg.data = 0/1/2
        """
        try:
            state = msg.data
            if state not in (0, 1, 2):
                self.get_logger().warn(f"Invalid LED state: {state}")
                return
            if self.mode == "AUTO":
                if state==0:
                    self.get_logger().info("Turn off LED light")
                if state==1:
                    self.get_logger().info("Flash once")
                if state==2:
                    self.get_logger().info("Flash Twice")

                with self.lock:
                    self.LED.send_command(state)
            
            if self.mode == "MANUAL":
                with self.lock:
                    self.LED.send_command(state)
        except Exception as e:
            self.get_logger().error(f"Exception in request callback {e}")

    def state_callback(self, msg):
        """Read robot's state, if it's in MANUAL, do red animation else take request normally"""
        try:
            if msg.data == "MANUAL":
                self.mode = "MANUAL"
            elif msg.data == "AUTO":
                self.mode = "AUTO"
            else:
                self.get_logger().warn(f"Unrecognized mode {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Exception in state callback function {e}")
    
    def destroy_node(self):
        self.get_logger().info("Shutting down LED node")
        try:
            self.LED.send_command(0)
            self.LED.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LEDNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C received")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
