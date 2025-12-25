#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench # Optional: Standard message for PID forces

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
        self.integral += error * dt
        i = self.ki * self.integral
        
        # Derivative
        d = self.kd * (error - self.last_error) / dt if dt > 0 else 0.0
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
        self.pid_debug_pub = self.create_publisher(Float32MultiArray, '/pid_debug', 10)

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
        self.current_vel_surge = 0.0   # m/s
        self.current_vel_sway = 0.0    # m/s
        
        self.target_heading = 0.0
        self.target_vel_surge = 0.0
        self.target_vel_sway = 0.0 
        
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz
        
        self.get_logger().info("PID Controller Node Integrated with Odometry.")

    def odom_callback(self, msg: Odometry):
        """Updates current state from Odometry message."""
        
        # Update Linear Velocities (Surge and Sway)
        # Assuming odom velocity is in the base_link frame
        self.current_vel_surge = msg.twist.twist.linear.x
        self.current_vel_sway = msg.twist.twist.linear.y

        # Update Orientation (Yaw)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Convert Quaternion to Euler Yaw
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self.current_heading = math.degrees(yaw_rad)
        
    def setpoint_cb(self, msg):
        """Expects [target_surge_vel, target_sway_vel, target_heading_deg]"""
        if len(msg.data) >= 3:
            self.target_vel_surge = msg.data[0]
            self.target_vel_sway = msg.data[1]
            self.target_heading = msg.data[2]

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: return
        self.last_time = now

        # 1. Heading Error Calculation (Wraps -180 to 180)
        error_yaw = self.target_heading - self.current_heading
        while error_yaw > 180: error_yaw -= 360
        while error_yaw < -180: error_yaw += 360

        # 2. Compute PID Outputs
        # Surge and Sway use velocity feedback
        surge_cmd = self.pid_surge.compute(self.target_vel_surge, self.current_vel_surge, dt)
        sway_cmd = self.pid_sway.compute(self.target_vel_sway, self.current_vel_sway, dt)
        
        # Yaw uses position feedback (heading)
        # Pass the calculated error directly as the 'setpoint' and 0 as 'measurement'
        yaw_cmd = self.pid_yaw.compute(error_yaw, 0.0, dt)

        # 3. Publish to Thruster/PWM Controller
        msg = Float32MultiArray()
        msg.data = [float(surge_cmd), float(sway_cmd), float(yaw_cmd)]
        self.pwm_pub.publish(msg)

        # Optional: Log current state for tuning
        # self.get_logger().info(f"Surge Error: {self.target_vel_surge - self.current_vel_surge:.2f} | Yaw Error: {error_yaw:.2f}")

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