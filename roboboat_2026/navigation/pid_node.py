#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench # Optional: Standard message for PID forces
from navigation.pure_pursuit_node import PurePursuitPlanner

class PIDController:
    """A simple PID helper class with output clamping and basic anti-windup."""
    def __init__(self, kp, ki, kd, output_limits=(-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limits = output_limits
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, setpoint, measurement, dt):
        error = setpoint - measurement
        
        # Proportional
        p = self.kp * error
        
        # Integral
        # [TODO] Clamp the self.integral value to output limits or implement an "anti-windup" check. 
        # Otherwise, integral term can grow unbounded upon sustained error.
        self.integral += error * dt
        i = self.ki * self.integral
        
        # Derivative
        d = self.kd * (error - self.last_error) / dt if dt > 0.0001 else 0.0
        self.last_error = error
        
        output = p + i + d
        # Clamp to limits
        return max(min(output, self.limits[1]), self.limits[0])

class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # --- Subscribers ---
        # Main Feedback Source: Odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Target Setpoints: [target_surge_vel, target_sway_vel, target_heading_deg]
        self.setpoint_sub = self.create_subscription(
            Float32MultiArray, 
            '/cmd_setpoint', 
            self.setpoint_cb, 
            10
        )

        # --- Publishers ---
        self.pwm_pub = self.create_publisher(Float32MultiArray, '/teensy/pwm', 10)
        # self.pid_debug_pub = self.create_publisher(Float32MultiArray, '/pid_debug', 10)

        # --- Planner Instance ---
        # Adjust lookahead (e.g., 1.0m) and wheelbase (e.g., 0.5m) as needed
        self.planner = PurePursuitPlanner(lookahead_dist=1.0, wheelbase=0.5)
        
        # --- Parameters ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp_surge', 1.0), ('ki_surge', 0.0), ('kd_surge', 0.1),
                ('kp_sway', 0.8),  ('ki_sway', 0.0),  ('kd_sway', 0.05),
                ('kp_yaw', 0.05),  ('ki_yaw', 0.0),   ('kd_yaw', 0.01)
            ]
        )

        # --- PID Instances ---
        self.pid_surge = PIDController(
            self.get_parameter('kp_surge').value,
            self.get_parameter('ki_surge').value,
            self.get_parameter('kd_surge').value
        )
        self.pid_sway = PIDController(
            self.get_parameter('kp_sway').value,
            self.get_parameter('ki_sway').value,
            self.get_parameter('kd_sway').value
        )
        self.pid_yaw = PIDController(
            self.get_parameter('kp_yaw').value,
            self.get_parameter('ki_yaw').value,
            self.get_parameter('kd_yaw').value
        )

        # --- State Variables ---
        self.current_heading = 0.0     # Degrees
        # currently not using velocity feedback, only pose
        # self.current_vel_surge = 0.0   # m/s
        # self.current_vel_sway = 0.0    # m/s
        self.current_pos_surge = 0.0   # m
        self.current_pos_sway = 0.0    # m
        
        self.target_heading = 0.0       # Degrees
        self.target_pos_surge = 0.0
        self.target_pos_sway = 0.0
        # self.target_vel_surge = 0.0
        # self.target_vel_sway = 0.0 
        
        # [TODO] Reset last_time to the current time inside the very first callback execution.
        # If the /odom message or the first timer loop takes a while to start, the first dt will be very large, 
        # causing a massive spike in the Integral and Derivative terms.
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz
        
        self.get_logger().info("PID Controller Node Integrated with Odometry.")

    def odom_callback(self, msg: Odometry):
        """Updates current state from Odometry message."""
        
        # Update Position (Surge and Sway)
        # Assuming odom position is in the base_link frame
        self.current_pos_surge = msg.pose.pose.position.x
        self.current_pos_sway = msg.pose.pose.position.y

        # Update Orientation (Yaw)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Update Covariance (if needed)
        # cvx = msg.pose.covariance[0]
        # cvy = msg.pose.covariance[7]
        # cvw = msg.pose.covariance[35]

        # Convert Quaternion to Euler Yaw
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self.current_heading = math.degrees(yaw_rad)
        
    def setpoint_cb(self, msg):
        """
        If msg.data is a flattened list of points [x1, y1, x2, y2...],
        convert it to a list of tuples and give it to the planner.
        """
        points = []
        for i in range(0, len(msg.data), 2):
            points.append((msg.data[i], msg.data[i+1]))
        
        if len(points) >= 1:
            # We add current position as the start to ensure a valid segment exists
            path = [(self.current_pos_surge, self.current_pos_sway)] + points
            self.planner.set_path(path)

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: return
        self.last_time = now

        # Skip if no path is set
        if not self.planner.path:
            return

        # --- GET PURE PURSUIT TARGETS ---
        # Create a simple object or tuple to match what the planner expects
        class Pose:
            def __init__(self, x, y):
                self.x = x
                self.y = y

        curr_pose = Pose(self.current_pos_surge, self.current_pos_sway)
        
        # Get dynamic targets from the planner
        target_v, target_h = self.planner.get_target_setpoints(curr_pose)

        # --- HEADING ERROR CALCULATION ---
        # Use target_h from planner instead of self.target_heading
        error_yaw = target_h - self.current_heading
        while error_yaw > 180: error_yaw -= 360
        while error_yaw < -180: error_yaw += 360

        # --- COMPUTE PID OUTPUTS ---
        
        # Surge: We use the target velocity as a "Position" setpoint for your PID.
        # Since your PID is -1 to 1, target_v (0.5) acts as a steady forward command.
        # As you get closer to the end of the path, you might want to scale this down.
        surge_cmd = self.pid_surge.compute(target_v, 0.0, dt)
        
        # Sway: Pure Pursuit doesn't natively use sway. Keep at 0 or use for correction.
        sway_cmd = 0.0 
        
        # Yaw: Use the error calculated from the planner's heading
        yaw_cmd = self.pid_yaw.compute(error_yaw, 0.0, dt)

        # --- PUBLISH ---
        msg = Float32MultiArray()
        msg.data = [float(surge_cmd), float(sway_cmd), float(yaw_cmd)]
        self.pwm_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()