#!/usr/bin/env python3
import rclpy
import math
import time
import json
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, String, Bool
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool

from roboboat_2026.util.helper import heading_error, get_heading_from_coords
from roboboat_2026.util import deviceHelper 
from roboboat_2026.api.util.gis_funcs import latlon2xy

class SimpleControl:
    def __init__(self):
        self.max_surge = 0.4
        self.max_yaw = 0.5
        self.last_surge = 0.0

    def control(self, distance, heading_error):
        surge = self.max_surge
        # if distance < 5.0:
        #     surge = max(distance / 4.0, 0.2)
        #     surge = min(surge,self.max_surge)

        yaw = 0.0
        if abs(heading_error) > 10:
            yaw = min(abs(heading_error) / 180.0 * self.max_yaw, self.max_yaw)
            yaw *= 1.0 if heading_error > 0 else -1.0

        # rate limit surge
        if surge - self.last_surge > 0.05:
            surge = self.last_surge + 0.05

        self.last_surge = surge
        surge = min(surge, self.max_surge)
        #print([surge,yaw])

        return surge, yaw


class WaypointFollowerService(Node):
    def __init__(self):
        super().__init__('waypoint_follower_service')

        self.controller = SimpleControl()
        self.position = None
        self.heading = None
        self.active = False
        self.reached_all = False
        self.config = deviceHelper.variables
        

        self.create_subscription(PoseStamped, '/fused/pose', self.pose_cb, 10)
        self.create_subscription(Float32MultiArray, '/GPS', self.gps_cb, 10)
        self.create_subscription(String, '/harbor_alert',self.alert_cb, 10)
        self.create_subscription(Float32MultiArray, '/nav2point',self.point_cb,10)
        

        # flags
        self.alert_detected = False
        self.alert_finished = False

        self.pwm_pub = self.create_publisher(Float32MultiArray, '/teensy/pwm', 10)
        self.task_pub = self.create_publisher(String, '/cur_task', 10)
        self.state_pub = self.create_publisher(Bool, '/WP_finished', 10)

        self.client = self.create_client(
            GetParameters,
            '/gps_fusion_node/get_parameters'
        )

        # request origin from ekf node
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for parameter service...')
        self.request = GetParameters.Request()
        self.request.names = ['origin']

        # Get origin
        self.origin = None
        self.create_subscription(Float32MultiArray, 'ekforigin', self.origin_cb, 1)
        self.waypoint_path = None
        self.create_subscription(String, '/waypoint_path', self.path_cb, 10)

        self.active_goal = None
        self.queue = []
        self.loop = self.create_timer(0.1, self.control_loop)   # 20Hz
        self.get_logger().info("Waypoint Follower Service Server ready")
        self.get_logger().info("Publish waypoint path to /waypoint_path")

    def path_cb(self, msg):
        self.waypoint_path = msg.data
        self.load_waypoints(path=self.waypoint_path)
        self.reached_all = False
        self.active = True
        self.get_logger().info(f"Waypoint path set to: {self.waypoint_path}")

    def pose_cb(self, msg):
        self.position = [msg.pose.position.x, msg.pose.position.y]

    def gps_cb(self, msg):
        self.heading = msg.data[2]

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

    def origin_cb(self, msg):
        if self.origin is None:
            self.origin = msg.data

    def point_cb(self,msg):
        """Point will be given in lat lon for waypoint nav"""
        lat, lon, task_idx = msg.data
        tasks = ['UNKNOWN','NONE','ENTRY_EXIT','NAV_CHANNEL','SPEED_CHALLENGE','OBJECT_DELIVERY','DOCKING','SOUND_SIGNAL']
        x,y = self.alert2xy(lat,lon)

        # override the current point
        self.reached_all = False
        self.active = True
        self.active_goal = [x,y,tasks[int(task_idx)]]
        

    def stop_vehicle(self):
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self.pwm_pub.publish(msg)

    def alert2xy(self,lat,lon):
        lat1 = math.radians(float(lat))
        lon1 = math.radians(float(lon))

        R = 6378137.0

        lat0 = math.radians(float(self.origin[0]))
        lon0 = math.radians(float(self.origin[1]))

        dlat = lat1 - lat0
        dlon = lon1 - lon0

        x = R * dlon * math.cos(lat0)
        y = R * dlat
        print(f"x is {x}")
        print(f"y is {y}")
        return x,y

    def latlon2xy(self):
        """Convert all lat/lon in self.queue to local XY (meters)"""
        print(self.origin)
        print(type(self.origin))
        print(self.origin[0])
        lat0 = math.radians(float(self.origin[0]))
        lon0 = math.radians(float(self.origin[1]))

        R = 6378137.0  # Earth radius in meters (WGS84)

        xy_queue = []

        for wp in self.queue:
            lat = math.radians(wp[0])
            lon = math.radians(wp[1])
            task = wp[2]

            dlat = lat - lat0
            dlon = lon - lon0

            x = R * dlon * math.cos(lat0)   # East
            y = R * dlat                    # North
            print(f"x is {x}")
            print(f"y is {y}")

            xy_queue.append([x, y, task])

        self.queue = xy_queue

    def load_waypoints(self, path):
        with open(path, 'r') as f:
            file = json.load(f)
            waypoints = file['waypoints']
            names = file['name']
            for wp in waypoints:
                self.queue.append([float(wp['lat']), float(wp['lon']),wp['task']])
                # self.latlon2xy()
                self.get_logger().info(f"Received waypoint: {wp}")
            
            self.latlon2xy()

    def control_loop(self):
        if self.reached_all:
            state_msg = Bool()
            state_msg.data = True
            self.state_pub.publish(state_msg)

        if not self.active:
            return
        
        if self.position is None or self.heading is None:
            return

        if self.active_goal is None and self.queue:
            self.active_goal = self.queue.pop(0)
            self.get_logger().info(f"Starting waypoint: {self.active_goal}")

        if self.active_goal is None:
            return
        
            
        x, y, task = self.active_goal

        # publish task msg
        task_msg = String()
        task_msg.data = task
        self.task_pub.publish(task_msg)

        dx = x - self.position[0]
        dy = y - self.position[1]
        distance = math.hypot(dx, dy)

        desire_heading = get_heading_from_coords(dx, dy)
        error_heading = heading_error(self.heading, desire_heading)

        tolerance = 1.5
        if task=="DOCKING":
            tolerance = 0.5

        # Goal reached
        if distance < 0.5:
            if self.alert_detected and not self.alert_finished:
                pwm = Float32MultiArray()
                pwm.data = [0.0, 0.0, 0.0]
                self.pwm_pub.publish(pwm)
                self.alert_finished = True
           
                time.sleep(10) # sleep for 10s show that we stay at harbor alert

            self.get_logger().info(f"Reached waypoint x={x}, y={y}")
            if self.queue:
                self.active_goal = self.queue.pop(0)
            else:
                self.active_goal = None
                self.active = False
                self.reached_all = True
                pwm = Float32MultiArray()
                pwm.data = [0.0, 0.0, 0.0]
                self.pwm_pub.publish(pwm)
                state_msg = Bool()
                state_msg.data = True
                self.state_pub.publish(state_msg)
            return
        
        self.get_logger().info(f"{distance:.2f} m to goal | heading error: {error_heading:.1f}",throttle_duration_sec = 1)
        surge, yaw = self.controller.control(distance, error_heading)
        pwm = Float32MultiArray()
        pwm.data = [float(surge), 0.0, float(yaw)]
        self.pwm_pub.publish(pwm)

    def execute_cb(self, request, response):
        if self.is_executing:
            response.success = False
            response.message = "Already executing waypoints"
            return response

        if self.waypoint_path is None:
            response.success = False
            response.message = "No waypoint path set. Publish to /waypoint_path first"
            return response

        self.is_executing = True
        
        try:
            waypoints = self.load_waypoints(self.waypoint_path)
            self.get_logger().info(f"Loaded {len(waypoints)} waypoints from {self.waypoint_path}")

            for wp in waypoints:
                desire_x = float(wp['x'])
                desire_y = float(wp['y'])
                task = wp['task']

                self.get_logger().info(f"Going to waypoint {wp}")
                
                # Publish current task
                task_msg = String()
                task_msg.data = task
                self.task_pub.publish(task_msg)

                reached = False
                while not reached:
                    rclpy.spin_once(self, timeout_sec=0.0)
                    if self.position is None or self.heading is None:
                        time.sleep(0.1)
                        self.get_logger().warn("Localization not received!")
                        continue

                    print(self.position)
                    print(self.heading)

                    dx = desire_x - self.position[0]
                    dy = desire_y - self.position[1]
                    distance = math.hypot(dx, dy)

                    desire_heading = get_heading_from_coords(dx, dy)
                    err = heading_error(self.heading, desire_heading)

                    self.get_logger().info(
                        f"Distance: {distance:.2f}m, Heading error: {err:.2f}°",
                        throttle_duration_sec=1.0
                    )

                    if distance < 0.5:
                        self.stop_vehicle()
                        reached = True
                    else:
                        surge, yaw = self.controller.control(distance, err)
                        pwm = Float32MultiArray()
                        pwm.data = [surge, 0.0, yaw]
                        self.pwm_pub.publish(pwm)

                    time.sleep(0.1)

            self.stop_vehicle()
            response.success = True
            response.message = f"All {len(waypoints)} waypoints completed"
            
        except Exception as e:
            self.stop_vehicle()
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(f"Waypoint following failed: {str(e)}")
        
        finally:
            self.is_executing = False

        return response
    
    def shutdown(self):
        """Custom shutdown logic"""
        self.get_logger().info("Shutdown FTP node started")

        if hasattr(self, 'loop'):
            self.loop.cancel()

        # stop motors, save logs, close files, etc.
        self.stop_vehicle()

        self.get_logger().info("Destroying node")
        super().destroy_node()


def main():
    rclpy.init()
    node = WaypointFollowerService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C received")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
