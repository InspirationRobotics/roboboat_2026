import rclpy
import time
import math
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, String, Bool
from nav_msgs.msg import OccupancyGrid
from sensor_msgs_py import point_cloud2
from threading import Thread, Lock
import numpy as np
from roboboat_2026.util.helper import haversine

class Navchannel(Node):
    def __init__(self):
        super().__init__('nav_channel')
        # Params
        self.declare_parameter('look_forward_distance', 1.5)   # meters, minimum of 1 meters
        self.declare_parameter('time_out', 120) # s
        self.declare_parameter('use_xy', True) # bool use xy or lat lon
        self.declare_parameter('end_point', [100,100]) # end point in either xy or lat lon
        self.declare_parameter('end_threshold', 5) # end point deteciton threshold in meters
        
        self.look_forward = self.get_parameter('look_forward_distance').value
        self.timeout = self.get_parameter('time_out').value
        self.use_xy = self.get_parameter('use_xy').value
        self.end_point = self.get_parameter('end_point').value
        self.end_threshold = self.get_parameter('end_threshold').value

        # Subscribers
        self.create_subscription(OccupancyGrid,'/lidar_map',self.map_cb,2)
        self.create_subscription(Float32MultiArray, '/GPS', self.gps_cb,2)
        self.create_subscription(PoseStamped, '/fused/pose', self.pose_cb, 1)
        self.create_subscription(Bool,'/NavChannel_start',self.active_cb, 1)

        # Publisher
        self.state_pub = self.create_publisher(Bool, '/NavChannel_finished', 10)
        self.pwm_pub = self.create_publisher(Float32MultiArray, '/teensy/pwm', 10)
        self.task_pub = self.create_publisher(String, '/cur_task', 10)

        # Store information
        self.pwms = [0.0,0.0,0.0]

        self.elapsed_time = 0 # track elapsed time
        self.position_xy = None # track position in x, y 
        self.position_latlon = None # track GPS position 
        
        self.origin = None
        self.grid = None # store occupancy grid

        # flags
        self.active = False
        self.reached_end = False
        self.mission_terminated = False

    
        # threads
        self.runner = Thread(target=self.run, daemon= True)
        self.runner.start()

    def active_cb(self,msg):
        self.get_logger().info("Begin mission")
        self.active = msg.data
        

    def pose_cb(self, msg):
        self.position_xy = [msg.pose.position.x, msg.pose.position.y]

    def map_cb(self, msg):
        width = msg.info.width
        height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        self.grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

    def gps_cb(self, msg):
        self.position_latlon = [msg.data[0],msg.data[1]]

    def sum_cost(self,side):
        # in ros2, x is forward
        x = self.grid.shape[1] // 2  # width/2
        y = self.grid.shape[0] // 2  # height/2


        # convert meters to cells
        dx  = self.look_forward // self.resolution
        dy  = dx
        deadzone = 1 // self.resolution # 1 m of deadzone in unit of cells
        if side=="left": 
            front_section = np.sum(self.grid[int(y+1):int(y+dy),int(x+1):int(x+dx)])
            side_section  = np.sum(self.grid[int(y+deadzone):int(y+dy),int(x-deadzone):int(x)])
            return front_section + side_section
        
        else:
            front_section = np.sum(self.grid[int(y-dy):int(y+1),int(x+1):int(x+dx)])
            side_section  = np.sum(self.grid[int(y-dy):int(y-deadzone),int(x-deadzone):int(x)])
            return front_section + side_section
        
    def run(self):
        def check_distance():
            if self.use_xy:
                dx = self.position_xy[0] - self.end_point[0]
                dy = self.position_xy[1] - self.end_point[1]
                delta_distance = math.sqrt(dx**2 + dy**2)
                return delta_distance
            else:
                d = haversine(self.position_latlon[0],self.position_latlon[1],self.end_point[0],self.end_point[1])
                return d

        def mission_terminate():
            # go forward for 3 s
            pwms = Float32MultiArray()
            pwms.data = [0.8,0.0,0.0]
            self.pwm_pub.publish(pwms)

            time.sleep(3)

            pwms = Float32MultiArray()
            pwms.data = [0.0,0.0,0.0]
            self.pwm_pub.publish(pwms)

            # tell mission planner we finished
            state_msg = Bool()
            state_msg.data = True
            self.state_pub.publish(state_msg)

            # deactivate the mission 
            self.active = False

        start_time = None
        print(start_time)
        print(self.active)
        while not self.mission_terminated:
            if self.active:
                if start_time == None:
                    start_time = time.time()
                    print(start_time)
                self.get_logger().info(f"Mission elapsed for {self.elapsed_time}")
                task_msg = String()
                task_msg.data = "NAV_CHANNEL"
                self.task_pub.publish(task_msg)

                # Check time out
                self.elapsed_time = time.time() - start_time
                if self.elapsed_time > self.timeout:
                    self.get_logger().info("Missino timed out")
                    self.mission_terminated = True
                    mission_terminate()
                    return
                
                # check localization and sensor topics
                # if self.grid is None or self.position_xy is None or self.position_latlon is None:
                #     self.get_logger().info("Localization not ready")
                #     time.sleep(1)
                #     continue

                # check finished
                # delta_distance = check_distance()
                # if delta_distance < self.end_threshold:
                #     self.get_logger().info("Reached end point")
                #     self.mission_terminated = True
                #     mission_terminate()
                #     return
                    
                
                # regular logic
                """Diving the area in front of the vehicle in to two section, 
                tell the boat to turn to the section with lower cost"""
                left_cost = self.sum_cost('left')
                right_cost = self.sum_cost('right')

                self.get_logger().info(f"left cost {left_cost}, right cost {right_cost}")
                
                if abs(left_cost-right_cost)/((left_cost+right_cost)/2) < 0.45:
                    # go straight
                    self.get_logger().info(f"cost difference is {abs(left_cost-right_cost)}, going forward")
                    pwm_msg = Float32MultiArray()
                    pwm_msg.data = [0.8,0.0,0.0]
                    self.pwm_pub.publish(pwm_msg)
                elif left_cost < right_cost:
                    # yaw left
                    self.get_logger().info("Yawing left")
                    pwm_msg = Float32MultiArray()
                    pwm_msg.data = [0.8,0.0,0.-0.15]
                    self.pwm_pub.publish(pwm_msg)
                else:
                    self.get_logger().info("Yawing right")
                    pwm_msg = Float32MultiArray()
                    pwm_msg.data = [0.8,0.0,0.15]
                    self.pwm_pub.publish(pwm_msg)
                
                time.sleep(0.1)
            else:
                self.get_logger().info("wait for activation")
                time.sleep(1)
def main():
    rclpy.init()
    node = Navchannel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
            

                

                



