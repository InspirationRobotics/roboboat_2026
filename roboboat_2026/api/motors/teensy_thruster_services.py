#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix, NavSatStatus
from threading import Thread, Lock
from roboboat_2026.util import deviceHelper
import serial
import time


class Teensy:
    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200):
        self.port = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        self.lock = Lock()

        # Preserve original behavior: on startup send a neutral command in the
        # "surge,sway,yaw,pump" format expected by the existing Teensy firmware.
        self.send_msg([0.0, 0.0, 0.0, 0])
        time.sleep(1)

    def send_msg(self, command):
        """Send the original comma-separated numeric command."""
        parsed = ",".join(str(x) for x in command) + "\n"
        with self.lock:
            self.port.write(parsed.encode())

    def send_line(self, line: str):
        """Send a raw command line (e.g. 'MOTOR,1,1600' or 'STOP')."""
        if not line.endswith("\n"):
            line += "\n"
        with self.lock:
            self.port.write(line.encode())

    def read_GPS(self):
        with self.lock:
            return self.port.readline()


# ==========================================================
#  ROS 2 Node
# ==========================================================

class TeensyNode(Node):
    def __init__(self):
        super().__init__('Teensy')

        self.config = deviceHelper.variables
        self.port = deviceHelper.dataFromConfig('teensy')
        self.baurd_rate = self.config.get('teensy').get('rate')

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
        self.teensy = Teensy(port=self.port, baudrate=self.baurd_rate)

        self.lock = Lock()
        self.cmd = [0.0, 0.0, 0.0]

        # RAW thruster test state (None = normal mixed control)
        self.raw_thruster_idx = None  # 1..4 when active
        self.raw_thruster_pwm = 1600

        self.loop = self.create_timer(0.05, self.write_loop)  # 20Hz

        # Subscribe to normalized surge/sway/yaw command
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/teensy/pwm',
            self.listener_callback,
            10
        )
        # to send in cli:
        # ros2 topic pub /teensy/pwm std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0]}"

        # Water pump toggle (kept identical)
        self.srv = self.create_service(
            Trigger,
            'toggle_water_pump',
            self.pump_callback
        )
        # to test service in cli:
        # ros2 service call /toggle_water_pump std_srvs/srv/Trigger "{}"

        # Additional: 4 thruster test toggles (do NOT affect normal behavior unless activated)
        self.thruster_srvs = []
        for i in range(1, 5):
            self.thruster_srvs.append(
                self.create_service(
                    Trigger,
                    f'toggle_thruster_{i}',
                    self._make_thruster_callback(i)
                )
            )
        # to test in cli:
        # ros2 service call /toggle_thruster_1 std_srvs/srv/Trigger "{}"
        # ros2 service call /toggle_thruster_2 std_srvs/srv/Trigger "{}"
        # ros2 service call /toggle_thruster_3 std_srvs/srv/Trigger "{}"
        # ros2 service call /toggle_thruster_4 std_srvs/srv/Trigger "{}"

    def _make_thruster_callback(self, thruster_idx: int):
        def cb(request, response):
            with self.lock:
                # toggle behavior:
                # - if currently testing this thruster -> stop test
                # - else -> start testing this thruster (and stop any other test)
                if self.raw_thruster_idx == thruster_idx:
                    self.raw_thruster_idx = None
                    # Tell Teensy to exit RAW mode (per your .ino sketch)
                    self.teensy.send_line("STOP")
                    response.success = True
                    response.message = f"Thruster {thruster_idx} test stopped (returned to mixed control)."
                    self.get_logger().info(response.message)
                else:
                    self.raw_thruster_idx = thruster_idx
                    # Immediately command it once; write_loop will keep it alive
                    self.teensy.send_line(f"MOTOR,{thruster_idx},{self.raw_thruster_pwm}")
                    response.success = True
                    response.message = f"Thruster {thruster_idx} test started at {self.raw_thruster_pwm} PWM."
                    self.get_logger().info(response.message)
            return response
        return cb

    def write_loop(self):
        """
        Preserves original behavior by default:
          sends [surge, sway, yaw, pump] at 20 Hz

        Only when a thruster test service is active does it send:
          'MOTOR,idx,1600' at 20 Hz
        """
        with self.lock:
            if self.raw_thruster_idx is not None:
                idx = self.raw_thruster_idx
                pwm = self.raw_thruster_pwm
                # Keep sending so the Teensy doesn't time out RAW mode
                self.teensy.send_line(f"MOTOR,{idx},{pwm}")
                return

            surge, sway, yaw = self.cmd
            pump = 1 if self.activate_pump else 0
            self.teensy.send_msg([surge, sway, yaw, pump])

    def listener_callback(self, msg):
        """Expect msg.data = [surge, sway, yaw], each in [-1, 1]."""
        if len(msg.data) != 3:
            self.get_logger().error("cmd_vel_normalized must be [surge, sway, yaw]")
            return

        surge = round(float(msg.data[0]), 6)
        sway = round(float(msg.data[1]), 6)
        yaw = round(float(msg.data[2]), 6)

        self.logger.debug(f"pwm: {[surge, sway, yaw]}")

        with self.lock:
            self.cmd = [surge, sway, yaw]

    def pump_callback(self, request, response):
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
        # If in RAW test mode, exit it cleanly first
        try:
            self.teensy.send_line("STOP")
        except Exception:
            pass
        self.teensy.send_msg([0, 0, 0, 0])
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
