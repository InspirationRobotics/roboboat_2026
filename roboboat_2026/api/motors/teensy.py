#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix, NavSatStatus
from threading import Thread, Lock
from roboboat_2026.util import deviceHelper
import serial
import time

class Teensy:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        self.port = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        self.lock = Lock()
        self.send_msg([0.0,0.0,0.0,0.0,0])  # esc 1-4,activate_pump (1 to activate 0 to deactivate)
        time.sleep(5)


    def send_msg(self, command):
        parsed = ",".join(str(x) for x in command) + "\n"
        with self.lock:
            self.port.write(str(parsed).encode())
        

    def read(self):
        with self.lock:
            return self.port.readline()

# ==========================================================
#  ROS 2 Node
# ==========================================================

class TeensyNode(Node):
    def __init__(self):
        super().__init__('Teensy')

        self.config = deviceHelper.variables
        self.port   = deviceHelper.dataFromConfig('teensy')
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
        self.teensy = Teensy(port=self.port,baudrate=self.baurd_rate)

        self.lock = Lock()
        self.cmd = [0.0,0.0,0.0]

        self.writer = self.create_timer(0.05, self.write_loop)  # 20Hz
        self.reader = self.create_timer(0.05, self.read_loop)   # 20Hz

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

        # publish robot state
        self.state_pub = self.create_publisher(
            String,
            '/robot_state',
            10
        )
        

    def write_loop(self):
        
        with self.lock:
            surge,sway,yaw = self.cmd  

        if deviceHelper.boat == "barco polo":
            m1 = -surge +sway +yaw
            m2 =  surge + sway + yaw
            m3 =  surge + sway - yaw
            m4 =  - surge + sway - yaw
        elif deviceHelper.boat == "crusader":
            m1 = -surge -sway +yaw
            m2 =  - surge - sway - yaw
            m3 =  surge - sway - yaw
            m4 =  surge - sway + yaw

        with self.lock:
            if self.activate_pump:
                self.teensy.send_msg([m1,m2,m3,m4,1])
                
            else:
                self.teensy.send_msg([m1,m2,m3,m4,0])
                
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

        self.logger.debug(f"pwm: {[surge,sway,yaw]}")
        
        with self.lock:
            self.cmd = [surge,sway,yaw]

    def pump_callback(self, request, response):
        # Do your reset logic here
        self.get_logger().info('Pump triggered')

        with self.lock:
            self.activate_pump = not self.activate_pump
        response.success = True
        response.message = 'Pump triggered successfully'
        return response

    def read_loop(self):
        line = ""
        try:
            raw = self.teensy.read()
            if not raw:
                return
            line = raw.decode("utf-8", errors="ignore")
            line = line.replace("\x00", "").strip()

            if not line:
                return 
            if not line.isdigit():
                self.get_logger().warning(f"Non-numeric line from teensy: {repr(line)}")
                return
            val = int(line)
            if val == 1:
                state = "AUTO"
            elif val == 0:
                state = "MANUAL"
            else:
                return 
            msg = String()
            msg.data = state
            self.state_pub.publish(msg)

        except Exception as e:
            self.get_logger().warning(f"Exception in read_loop, line is {repr(line)}")
            self.get_logger().error(str(e))

    def destroy_node(self):
        self.get_logger().info("Stopping thrusters...")
        self.teensy.send_msg([0,0,0,0])
        # self.GPSThread.join(timeout=1) no GPS for barco Polo
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
