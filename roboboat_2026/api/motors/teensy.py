#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix, NavSatStatus
from threading import Thread, Lock
import serial
import time


class Teensy:
    def __init__(self, port="/dev/ttyACM1", baudrate=115200):
        self.port = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        self.lock = Lock()
        self.send_msg([0,0,0,0])  # surge,sway,yaw,activate_pump (1 to activate 0 to deactivate)
        time.sleep(1)


    def send_msg(self, command):
        parsed = ",".join(str(x) for x in command) + "\n"
        with self.lock:
            self.port.write(parsed.encode())
        

    def read_GPS(self):
        with self.lock:
            return self.port.readline()

# ==========================================================
#  ROS 2 Node
# ==========================================================

class TeensyNode(Node):
    def __init__(self):
        super().__init__('Teensy')

        # GPS data
        self.lat = 0.0
        self.lon = 0.0
        self.heading = 0.0
        self.velocity = 0.0

        # pump
        self.activate_pump = False
        
        self.logger = self.get_logger()
        self.logger.info("Thruster Controller Node Started")

        # Thruster driver
        self.teensy = Teensy()
        self.GPSThread = Thread(target=self.readloop, daemon=True)
        self.lock = Lock()

        # Subscribe to normalized surge/sway/yaw command
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/teensy/pwm',
            self.listener_callback,
            10
        )   # to send in cli: ros2 topic pub /teensy/pwm std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0]}"

        self.srv = self.create_service(
            Trigger,
            'toggle_water_pump',
            self.pump_callback
        )
        # to test service in cli: ros2 service call /toggle_water_pump std_srvs/srv/Trigger "{}"

        # self.GPSThread.start() No GPS reading for Barco Polo

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
        with self.lock:
            if self.activate_pump:
                self.teensy.send_msg([surge,sway,yaw,1])
                
            else:
                self.teensy.send_msg([surge,sway,yaw,0])

    def pump_callback(self, request, response):
        # Do your reset logic here
        self.get_logger().info('Pump triggered')

        with self.lock:
            self.activate_pump = not self.activate_pump
        response.success = True
        response.message = 'Pump triggered successfully'
        return response
    
    def parseGPS(self, line: str):
        """Parse GPS data lat,lon,heading,velocity with error handling"""
        try:
            parts = line.split(',')
            if len(parts) != 4:
                raise ValueError(f"Expected 4 values, got {len(parts)}")

            self.lat, self.lon, self.heading, self.velocity = map(float, parts)

            # Publish NavSatFix
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps"
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.latitude = self.lat
            msg.longitude = self.lon
            msg.altitude = 0.0
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.publisher.publish(msg)

            # Publish heading
            heading_msg = Float32()
            heading_msg.data = self.heading
            self.heading_pub.publish(heading_msg)

        except ValueError as ve:
            self.get_logger().warn(f"Invalid GPS data format: '{line}' | {ve}")
        except Exception as e:
            self.get_logger().warn(f"Unexpected error parsing GPS line: '{line}' | {e}")

    def readloop(self):
        while rclpy.ok():
            line = self.teensy.read_GPS().decode('utf-8').strip()
            if line:
                try:
                    self.parseGPS(line)
                except Exception as e:
                    self.get_logger().warn(f"Failed to parse GPS line: {line} | {e}")
            time.sleep(0.5)

    def destroy_node(self):
        self.get_logger().info("Stopping thrusters...")
        self.teensy.send_PWM([0,0,0])
        self.GPSThread.join(timeout=1)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TeensyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C received")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
