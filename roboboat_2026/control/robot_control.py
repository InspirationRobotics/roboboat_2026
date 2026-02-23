import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import TwistStamped
from roboboat_2026.util import deviceHelper
from roboboat_2026.util.helper import heading_error, get_distance, get_heading_from_coords
from simple_pid import PID
import geometry_msgs.msg


import threading
import time
import numpy as np
import math


class RobotControl(Node):

    def __init__(self):
        super().__init__('robot_control_node')

        self.config = deviceHelper.variables
        self.lock = threading.Lock()

        # flags
        self.activate_pump = False
        self.prev_state = None
        
        # robot state
        self.state = "MANUAL"  # "HEADING_HOLD" 
        self.position = []
        self.velocity = []
        self.heading  = None

        # control
        self.desire_heading = None
        self.direct_input = [0] * 3
        

        # subscriber
        self.odom_sub = self.create_subscription(
            Odometry, '/fused/odometry', self.odom_cb,10
        )

        self.gps_sub = self.create_subscription(
            Float32MultiArray,
            '/GPS', 
            self.gps_cb,
            10
        )

        self.heading_pid = PID(
                self.config.get("YAW_PID_P", 12),
                self.config.get("YAW_PID_I", 0.01),
                self.config.get("YAW_PID_D", 0.0),
                setpoint=0,
                output_limits=(-1, 1),
            )

        # control loop
        self.timer = self.create_timer(0.02, self.control_loop)

        # publisher
        self.pub = self.create_publisher(Float32MultiArray, "/teensy/pwm", 10)


    """Callbacks"""
    def odom_cb(self, msg):
        with self.lock:
            self.position = [msg.pose.position.x,msg.pose.position.y]
            self.velocity = [msg.twist.linear.x, msg.twist.linear.y]

    def gps_cb(self, msg):
        with self.lock:
            self.heading = msg.data[2]  # extract heading only


    def origin_cb(self, msg):
        if self.origin is None:
            self.origin = msg.data

    def alert_cb(self, msg):
        self.alert_detected = True
        line = msg.data
        lines = line.strip().split(',')
        if lines[0]=="0":
            harbor_pos = self.config.get('harbor_pos').get("one_blast")
        elif lines[0] == "1":
            harbor_pos = self.config.get('harbor_pos').get("one_blast")
        elif lines[0] == "2":
            harbor_pos = self.config.get('harbor_pos').get("two_blast")
        else:
            self.get_logger().warn("INVALID ALERT MSG")
            return
        print(harbor_pos)

        x,y = self.alert2xy(harbor_pos[0],harbor_pos[1])
        self.active = False
        self.active_goal = [x,y,"SOUND_SIGNAL"]
        self.active = True
        self.alert_detected = True
    
    """Setters"""
    def set_state(self, next_state:str):
        """
        Args:
            next_state(str): MANUAL or HEADING_HOLD
        """
        # check if state exist
        if next_state not in ['MANUAL','HEADING_HOLD']:
            self.get_logger().warning(f"{next_state} does not exist!")
            return
        with self.lock:
            if next_state=="HEADING_HOLD":
                self.reset()

            self.prev_state = self.state
            self.state = next_state 

    def set_heading(self, desire_heading):
        """When you use set heading, you will enter heading hold mode"""
        with self.lock:
            self.desire_heading = desire_heading
            self.set_state("HEADING_HOLD")

    """Servos"""
    def set_pump_state(self, state):
        """
        Args:
            state (string): on or off
        """
        if state=="on":
            self.activate_pump = True
        elif state == "off":
            self.activate_pump = False
        else:
            self.get_logger().warning(f"Not a valid state")

    def launch_racquetball(self):
        pass

    """Simple movements"""
    def stop_vehicle(self):
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self.pwm_pub.publish(msg)

    def go_to_abs_heading(self, desire_heading):
        # set to manual state
        prev_state = self.state
        self.set_heading(desire_heading)
        self.set_state("HEADING_HOLD")
        start_time = time.time()
        while time.time()-start_time <30: # 30 seconds timeout
            rclpy.spin_once(self, timeout_sec=0.0)
            error = heading_error(self.heading, self.desire_heading)

            if abs(error) <= 5:  # 5 degrees tolerance
                self.get_logger().info(f"Reached {desire_heading}")
                break

            time.sleep(0.1)
        
        # set back to previous state
        self.set_state(prev_state)
    
    def go_to_rel_heading(self, relative_heading):
        desire_heading = self.heading + relative_heading
        self.go_to_abs_heading(desire_heading)
        
    def go_forward(self, distance):
        """
        Going forward, 
        
        :param distance: forward distance, can be negative
        """

        init_position = self.position
        moved_distance = get_distance((init_position[0],init_position[1]),(self.position[0],self.position[1]))
        while abs(moved_distance) - 0.5 < distance:  # 0.5 m tolerance
            rclpy.spin_once(self, timeout_sec=0.0)
            if distance > 0:
                self.movement(surge=0.6)
            else:
                self.movement(surge=-0.6)
            moved_distance = get_distance((init_position[0],init_position[1]),(self.position[0],self.position[1]))
            time.sleep(0.1)

        self.get_logger().info(f"Moved forward {distance} m")


    def go_lateral(self, distance):
        """
        Going lateral, state will be in heading hold
        
        :param distance: forward distance, can be negative
        """
        self.set_heading(self.heading)
        self.set_state("HEADING_HOLD")
        init_position = self.position
        moved_distance = get_distance((init_position[0],init_position[1]),(self.position[0],self.position[1]))
        while abs(moved_distance) - 0.5 < distance:  # 0.5 m tolerance
            rclpy.spin_once(self, timeout_sec=0.0)
            if distance > 0:
                self.movement(sway=0.6)
            else:
                self.movement(sway=-0.6)
            moved_distance = get_distance((init_position[0],init_position[1]),(self.position[0],self.position[1]))
            time.sleep(0.1)

        self.get_logger().info(f"Moved lateral {distance} m")

    def go_waypoint(self, x, y):
        #TODO publish desire waypoints to nav2
        pass


    def reset(self):
        self.desire_heading = None
        self.direct_input = [0] * 3
        self.heading_pid.reset()
    
    """Control loop"""
    def control_loop(self):
        """Control loop has two different state, MANUAL or HEADING_HOLD, each has different capability"""
        if self.state == "MANUAL":
            self.__movement(self.direct_input[0],self.direct_input[1],self.direct_input[2])
        elif self.state == "HEADING_HOLD":
            if self.desire_heading is None:
                self.desire_heading = self.heading
            error = heading_error(self.heading, self.desire_heading)
            heading_ctr = self.heading_pid(-error / 180)

            self.__movement(self.direct_input[0],self.direct_input[1],heading_ctr)

    """Control the thrusters"""
    def movement(self, surge=0, sway=0, yaw=0):
        ctr = [0,0,0]
        ctr[0] = surge
        ctr[1] = sway
        ctr[2] = yaw
        with self.lock:
            self.direct_input = ctr

    def __movement(self, surge = 0, sway = 0, yaw = 0):
        surge = np.clip(surge, -1,1)
        sway  = np.clip(sway, -1, 1)
        yaw   = np.clip(yaw, -1,1)

        ctr = [0,0,0,0]
        if self.activate_pump:
            ctr = [surge, sway, yaw, 1]
        else:
            ctr = [surge, sway, yaw, 0]

        msg = Float32MultiArray()
        msg.data = ctr

        self.pub.publish(msg)

    