#!/usr/bin/env python3
"""
Unicycle Controller Template for TurtleBot3
============================================
A clean template for unicycle model control with position and velocity feedback.

Unicycle Model:
    ẋ = v * cos(θ)
    ẏ = v * sin(θ)
    θ̇ = ω

Where:
    (x, y) = position in global frame
    θ = orientation (yaw) in global frame
    v = linear velocity (m/s)
    ω = angular velocity (rad/s)

Author: Custom implementation for TurtleBot3
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class UnicycleController(Node):
    """
    Unicycle controller for a single TurtleBot3.
    
    Provides:
    - Unicycle velocity command interface (v, ω)
    - Global frame position feedback (x, y, θ)
    - Global frame velocity feedback (vx, vy, ω)
    """

    def __init__(self, robot_namespace: str = ''):
        """
        Initialize the unicycle controller.
        
        Args:
            robot_namespace: Robot namespace (e.g., 'tb3_1' for multi-robot, 
                           empty string '' for single robot)
        """
        super().__init__('unicycle_controller')
        
        # Robot namespace configuration
        self.namespace = robot_namespace
        prefix = f'/{robot_namespace}' if robot_namespace else ''
        
        # ============================================
        # Robot State Variables (Global Frame)
        # ============================================
        
        # Position in global frame
        self.x = 0.0          # x position (m)
        self.y = 0.0          # y position (m)
        self.theta = 0.0      # orientation/yaw (rad)
        
        # Velocity in global frame
        self.vx = 0.0         # x velocity (m/s)
        self.vy = 0.0         # y velocity (m/s)
        self.omega = 0.0      # angular velocity (rad/s)
        
        # Velocity in body frame (from odometry)
        self.v_body = 0.0     # linear velocity in body frame (m/s)
        self.omega_body = 0.0 # angular velocity in body frame (rad/s)
        
        # Data validity flag
        self.odom_received = False
        
        # ============================================
        # ROS2 Publishers and Subscribers
        # ============================================
        
        qos = QoSProfile(depth=10)
        
        # Velocity command publisher (cmd_vel)
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            f'{prefix}/cmd_vel', 
            qos
        )
        
        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            f'{prefix}/odom',
            self.odom_callback,
            qos
        )
        
        # ============================================
        # Control Loop Timer (Optional)
        # ============================================
        
        self.control_rate = 50.0  # Hz
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, 
            self.control_loop
        )
        
        # Target velocities for control loop
        self.target_v = 0.0
        self.target_omega = 0.0
        
        self.get_logger().info(f'Unicycle Controller initialized')
        self.get_logger().info(f'  Topics: {prefix}/cmd_vel, {prefix}/odom')

    # ============================================
    # ODOMETRY CALLBACK - State Feedback
    # ============================================
    
    def odom_callback(self, msg: Odometry):
        """
        Process odometry message and update robot state.
        
        Updates:
        - Position (x, y, θ) in global frame
        - Velocity (vx, vy, ω) in global frame
        """
        # Extract position (global frame)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extract orientation (quaternion to yaw)
        q = msg.pose.pose.orientation
        self.theta = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        
        # Extract body frame velocities
        self.v_body = msg.twist.twist.linear.x
        self.omega_body = msg.twist.twist.angular.z
        self.omega = self.omega_body  # Angular velocity is same in both frames
        
        # Convert body velocity to global frame velocity
        # vx = v * cos(θ), vy = v * sin(θ)
        self.vx = self.v_body * math.cos(self.theta)
        self.vy = self.v_body * math.sin(self.theta)
        
        self.odom_received = True

    # ============================================
    # UNICYCLE CONTROL COMMAND
    # ============================================
    
    def set_velocity(self, v: float, omega: float):
        """
        Set unicycle velocity command.
        
        This is the main control interface for the unicycle model.
        
        Args:
            v: Linear velocity (m/s) - forward/backward motion
            omega: Angular velocity (rad/s) - rotation (positive = counter-clockwise)
        
        Unicycle Model:
            ẋ = v * cos(θ)
            ẏ = v * sin(θ)
            θ̇ = ω
        """
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = float(omega)
        
        self.cmd_vel_pub.publish(cmd)
        
        # Update targets for monitoring
        self.target_v = v
        self.target_omega = omega

    def stop(self):
        """Stop the robot (zero velocity command)."""
        self.set_velocity(0.0, 0.0)

    # ============================================
    # STATE GETTERS - Position Feedback
    # ============================================
    
    def get_position(self) -> tuple:
        """
        Get robot position in global frame.
        
        Returns:
            tuple: (x, y, theta) where:
                x: x-position (m)
                y: y-position (m)
                theta: orientation/yaw (rad), range [-π, π]
        """
        return (self.x, self.y, self.theta)
    
    def get_position_array(self) -> np.ndarray:
        """
        Get robot position as numpy array.
        
        Returns:
            np.ndarray: [x, y, theta]
        """
        return np.array([self.x, self.y, self.theta])

    # ============================================
    # STATE GETTERS - Velocity Feedback
    # ============================================
    
    def get_velocity_global(self) -> tuple:
        """
        Get robot velocity in global frame.
        
        Returns:
            tuple: (vx, vy, omega) where:
                vx: x-velocity (m/s)
                vy: y-velocity (m/s)
                omega: angular velocity (rad/s)
        """
        return (self.vx, self.vy, self.omega)
    
    def get_velocity_global_array(self) -> np.ndarray:
        """
        Get robot velocity in global frame as numpy array.
        
        Returns:
            np.ndarray: [vx, vy, omega]
        """
        return np.array([self.vx, self.vy, self.omega])
    
    def get_velocity_body(self) -> tuple:
        """
        Get robot velocity in body frame.
        
        Returns:
            tuple: (v, omega) where:
                v: linear velocity (m/s)
                omega: angular velocity (rad/s)
        """
        return (self.v_body, self.omega)

    # ============================================
    # UTILITY FUNCTIONS
    # ============================================
    
    @staticmethod
    def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        """
        Convert quaternion to yaw angle.
        
        Args:
            qx, qy, qz, qw: Quaternion components
            
        Returns:
            float: Yaw angle (rad), range [-π, π]
        """
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize angle to [-π, π].
        
        Args:
            angle: Input angle (rad)
            
        Returns:
            float: Normalized angle (rad)
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ============================================
    # CONTROL LOOP (User Implementation Area)
    # ============================================
    
    def control_loop(self):
        """
        Main control loop - runs at control_rate Hz.
        
        *** IMPLEMENT YOUR CONTROL LAW HERE ***
        
        Available state feedback:
            Position: self.x, self.y, self.theta
            Velocity: self.vx, self.vy, self.omega (global frame)
                      self.v_body, self.omega_body (body frame)
        
        Send commands using:
            self.set_velocity(v, omega)
        """
        # Example: Just maintain the target velocity (open loop)
        # Replace this with your control law
        pass

    # ============================================
    # EXAMPLE CONTROL METHODS
    # ============================================
    
    def go_to_point(self, target_x: float, target_y: float, 
                    k_linear: float = 0.5, k_angular: float = 1.0,
                    distance_threshold: float = 0.05) -> bool:
        """
        Simple proportional controller to go to a target point.
        
        Args:
            target_x: Target x position (m)
            target_y: Target y position (m)
            k_linear: Linear velocity gain
            k_angular: Angular velocity gain
            distance_threshold: Distance at which to consider goal reached (m)
            
        Returns:
            bool: True if goal reached, False otherwise
        """
        # Calculate error
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < distance_threshold:
            self.stop()
            return True
        
        # Desired heading
        desired_theta = math.atan2(dy, dx)
        theta_error = self.normalize_angle(desired_theta - self.theta)
        
        # Proportional control
        v = k_linear * distance
        omega = k_angular * theta_error
        
        # Apply velocity limits
        v = min(v, 0.22)  # TurtleBot3 max linear velocity
        omega = max(min(omega, 2.84), -2.84)  # TurtleBot3 max angular velocity
        
        self.set_velocity(v, omega)
        return False

    def print_state(self):
        """Print current robot state for debugging."""
        self.get_logger().info(
            f'Position: ({self.x:.3f}, {self.y:.3f}, {math.degrees(self.theta):.1f}°) | '
            f'Velocity: ({self.vx:.3f}, {self.vy:.3f}, {self.omega:.3f})'
        )


# ============================================
# MAIN FUNCTION
# ============================================

def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    # Create controller
    # For single robot: UnicycleController('')
    # For multi-robot: UnicycleController('tb3_1')
    controller = UnicycleController('')
    
    try:
        controller.get_logger().info('Waiting for odometry data...')
        
        # Wait for first odometry message
        while not controller.odom_received:
            rclpy.spin_once(controller, timeout_sec=0.1)
        
        controller.get_logger().info('Odometry received! Controller ready.')
        controller.get_logger().info('Press Ctrl+C to stop.')
        
        # ============================================
        # USER CODE: Add your control logic here
        # ============================================
        
        # Example: Move forward for 2 seconds then stop
        # controller.set_velocity(0.1, 0.0)
        
        # Example: Go to point (1.0, 1.0)
        # while rclpy.ok():
        #     rclpy.spin_once(controller, timeout_sec=0.02)
        #     if controller.go_to_point(1.0, 1.0):
        #         controller.get_logger().info('Goal reached!')
        #         break
        
        # Default: Just spin and let control_loop handle things
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down...')
        controller.stop()
    
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
