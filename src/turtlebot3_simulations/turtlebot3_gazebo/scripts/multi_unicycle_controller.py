#!/usr/bin/env python3
"""
Multi-Robot Unicycle Controller Template for TurtleBot3
=======================================================
A clean template for controlling 4 TurtleBots independently using unicycle model.

Each robot has independent:
- Velocity commands (v, ω)
- Position feedback (x, y, θ) in global frame
- Velocity feedback (vx, vy, ω) in global frame

Unicycle Model (per robot):
    ẋ = v * cos(θ)
    ẏ = v * sin(θ)
    θ̇ = ω

Author: Custom implementation for TurtleBot3
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from typing import Dict, List, Tuple, Optional


class RobotState:
    """State container for a single robot."""
    
    def __init__(self, name: str):
        self.name = name
        
        # Position in global frame
        self.x = 0.0          # x position (m)
        self.y = 0.0          # y position (m)
        self.theta = 0.0      # orientation/yaw (rad)
        
        # Velocity in global frame
        self.vx = 0.0         # x velocity (m/s)
        self.vy = 0.0         # y velocity (m/s)
        self.omega = 0.0      # angular velocity (rad/s)
        
        # Velocity in body frame
        self.v_body = 0.0     # linear velocity (m/s)
        self.omega_body = 0.0 # angular velocity (rad/s)
        
        # Target velocities
        self.target_v = 0.0
        self.target_omega = 0.0
        
        # Data validity
        self.odom_received = False


class MultiUnicycleController(Node):
    """
    Unicycle controller for multiple TurtleBot3 robots.
    
    Provides independent control for each robot with:
    - Unicycle velocity command interface (v, ω)
    - Global frame position feedback (x, y, θ)
    - Global frame velocity feedback (vx, vy, ω)
    """

    def __init__(self, robot_names: List[str] = None, namespace_prefix: str = 'tb3'):
        """
        Initialize the multi-robot unicycle controller.
        
        Args:
            robot_names: List of robot names. If None, defaults to ['tb3_1', 'tb3_2', 'tb3_3', 'tb3_4']
            namespace_prefix: Prefix for auto-generated names (used if robot_names is None)
        """
        super().__init__('multi_unicycle_controller')
        
        # Default robot configuration
        if robot_names is None:
            self.num_robots = 4
            self.robot_names = [f'{namespace_prefix}_{i+1}' for i in range(self.num_robots)]
        else:
            self.robot_names = robot_names
            self.num_robots = len(robot_names)
        
        # ============================================
        # Robot State Storage
        # ============================================
        
        self.robots: Dict[str, RobotState] = {}
        for name in self.robot_names:
            self.robots[name] = RobotState(name)
        
        # ============================================
        # ROS2 Publishers and Subscribers
        # ============================================
        
        qos = QoSProfile(depth=10)
        
        self.cmd_vel_pubs: Dict[str, any] = {}
        self.odom_subs: Dict[str, any] = {}
        
        for name in self.robot_names:
            # Velocity command publisher
            self.cmd_vel_pubs[name] = self.create_publisher(
                Twist,
                f'/{name}/cmd_vel',
                qos
            )
            
            # Odometry subscriber
            self.odom_subs[name] = self.create_subscription(
                Odometry,
                f'/{name}/odom',
                self._create_odom_callback(name),
                qos
            )
        
        # ============================================
        # Control Loop Timer
        # ============================================
        
        self.control_rate = 50.0  # Hz
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )
        
        self.get_logger().info(f'Multi-Unicycle Controller initialized for {self.num_robots} robots')
        self.get_logger().info(f'  Robots: {self.robot_names}')

    def _create_odom_callback(self, robot_name: str):
        """Create odometry callback closure for a specific robot."""
        def callback(msg: Odometry):
            self._odom_callback(msg, robot_name)
        return callback

    # ============================================
    # ODOMETRY CALLBACK - State Feedback
    # ============================================
    
    def _odom_callback(self, msg: Odometry, robot_name: str):
        """
        Process odometry message and update robot state.
        
        Args:
            msg: Odometry message
            robot_name: Name of the robot
        """
        robot = self.robots[robot_name]
        
        # Extract position (global frame)
        robot.x = msg.pose.pose.position.x
        robot.y = msg.pose.pose.position.y
        
        # Extract orientation (quaternion to yaw)
        q = msg.pose.pose.orientation
        robot.theta = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        
        # Extract body frame velocities
        robot.v_body = msg.twist.twist.linear.x
        robot.omega_body = msg.twist.twist.angular.z
        robot.omega = robot.omega_body
        
        # Convert body velocity to global frame
        robot.vx = robot.v_body * math.cos(robot.theta)
        robot.vy = robot.v_body * math.sin(robot.theta)
        
        robot.odom_received = True

    # ============================================
    # UNICYCLE CONTROL COMMANDS
    # ============================================
    
    def set_velocity(self, robot_name: str, v: float, omega: float):
        """
        Set unicycle velocity command for a specific robot.
        
        Args:
            robot_name: Name of the robot (e.g., 'tb3_1')
            v: Linear velocity (m/s)
            omega: Angular velocity (rad/s)
        """
        if robot_name not in self.cmd_vel_pubs:
            self.get_logger().warn(f'Unknown robot: {robot_name}')
            return
        
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        
        self.cmd_vel_pubs[robot_name].publish(cmd)
        
        # Update targets
        self.robots[robot_name].target_v = v
        self.robots[robot_name].target_omega = omega

    def set_velocities(self, velocities: Dict[str, Tuple[float, float]]):
        """
        Set velocities for multiple robots at once.
        
        Args:
            velocities: Dict mapping robot_name -> (v, omega)
                        e.g., {'tb3_1': (0.2, 0.0), 'tb3_2': (0.1, 0.5)}
        """
        for robot_name, (v, omega) in velocities.items():
            self.set_velocity(robot_name, v, omega)

    def set_all_velocities(self, v: float, omega: float):
        """
        Set the same velocity for all robots.
        
        Args:
            v: Linear velocity (m/s)
            omega: Angular velocity (rad/s)
        """
        for name in self.robot_names:
            self.set_velocity(name, v, omega)

    def stop(self, robot_name: str):
        """Stop a specific robot."""
        self.set_velocity(robot_name, 0.0, 0.0)

    def stop_all(self):
        """Stop all robots."""
        for name in self.robot_names:
            self.stop(name)

    # ============================================
    # STATE GETTERS - Single Robot
    # ============================================
    
    def get_position(self, robot_name: str) -> Tuple[float, float, float]:
        """
        Get position of a specific robot in global frame.
        
        Args:
            robot_name: Name of the robot
            
        Returns:
            tuple: (x, y, theta)
        """
        robot = self.robots[robot_name]
        return (robot.x, robot.y, robot.theta)
    
    def get_position_array(self, robot_name: str) -> np.ndarray:
        """
        Get position as numpy array.
        
        Returns:
            np.ndarray: [x, y, theta]
        """
        robot = self.robots[robot_name]
        return np.array([robot.x, robot.y, robot.theta])

    def get_velocity_global(self, robot_name: str) -> Tuple[float, float, float]:
        """
        Get velocity of a specific robot in global frame.
        
        Args:
            robot_name: Name of the robot
            
        Returns:
            tuple: (vx, vy, omega)
        """
        robot = self.robots[robot_name]
        return (robot.vx, robot.vy, robot.omega)
    
    def get_velocity_global_array(self, robot_name: str) -> np.ndarray:
        """
        Get velocity in global frame as numpy array.
        
        Returns:
            np.ndarray: [vx, vy, omega]
        """
        robot = self.robots[robot_name]
        return np.array([robot.vx, robot.vy, robot.omega])

    def get_velocity_body(self, robot_name: str) -> Tuple[float, float]:
        """
        Get velocity of a specific robot in body frame.
        
        Returns:
            tuple: (v, omega)
        """
        robot = self.robots[robot_name]
        return (robot.v_body, robot.omega)

    # ============================================
    # STATE GETTERS - All Robots
    # ============================================
    
    def get_all_positions(self) -> Dict[str, Tuple[float, float, float]]:
        """
        Get positions of all robots.
        
        Returns:
            Dict mapping robot_name -> (x, y, theta)
        """
        return {name: self.get_position(name) for name in self.robot_names}
    
    def get_all_positions_array(self) -> np.ndarray:
        """
        Get all positions as a numpy array.
        
        Returns:
            np.ndarray: Shape (num_robots, 3) with [x, y, theta] per row
        """
        return np.array([self.get_position_array(name) for name in self.robot_names])

    def get_all_velocities_global(self) -> Dict[str, Tuple[float, float, float]]:
        """
        Get velocities of all robots in global frame.
        
        Returns:
            Dict mapping robot_name -> (vx, vy, omega)
        """
        return {name: self.get_velocity_global(name) for name in self.robot_names}
    
    def get_all_velocities_global_array(self) -> np.ndarray:
        """
        Get all velocities as a numpy array.
        
        Returns:
            np.ndarray: Shape (num_robots, 3) with [vx, vy, omega] per row
        """
        return np.array([self.get_velocity_global_array(name) for name in self.robot_names])

    # ============================================
    # DIRECT STATE ACCESS (Convenience)
    # ============================================
    
    def __getitem__(self, robot_name: str) -> RobotState:
        """
        Direct access to robot state using indexing.
        
        Usage:
            controller['tb3_1'].x  # Get x position
            controller['tb3_1'].vx # Get x velocity
        """
        return self.robots[robot_name]

    # ============================================
    # UTILITY FUNCTIONS
    # ============================================
    
    @staticmethod
    def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        """Convert quaternion to yaw angle."""
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-π, π]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def is_ready(self) -> bool:
        """Check if all robots have received odometry."""
        return all(robot.odom_received for robot in self.robots.values())
    
    def ready_count(self) -> int:
        """Get number of robots with odometry data."""
        return sum(1 for robot in self.robots.values() if robot.odom_received)

    # ============================================
    # CONTROL LOOP (User Implementation Area)
    # ============================================
    
    def control_loop(self):
        """
        Main control loop - runs at control_rate Hz.
        
        *** IMPLEMENT YOUR CONTROL LAW HERE ***
        
        Each robot has independent state access:
            Position: self.robots['tb3_1'].x, .y, .theta
            Velocity: self.robots['tb3_1'].vx, .vy, .omega (global)
                      self.robots['tb3_1'].v_body, .omega_body (body)
        
        Or use convenience methods:
            self.get_position('tb3_1') -> (x, y, theta)
            self.get_velocity_global('tb3_1') -> (vx, vy, omega)
        
        Send commands:
            self.set_velocity('tb3_1', v, omega)
        """
        # Example: Implement your parallel control law for all robots here
        # 
        # for name in self.robot_names:
        #     x, y, theta = self.get_position(name)
        #     # Compute control for each robot independently
        #     v, omega = self.compute_control(name, x, y, theta)
        #     self.set_velocity(name, v, omega)
        pass

    # ============================================
    # EXAMPLE CONTROL METHODS
    # ============================================
    
    def go_to_point(self, robot_name: str, target_x: float, target_y: float,
                    k_linear: float = 0.5, k_angular: float = 1.0,
                    distance_threshold: float = 0.05) -> bool:
        """
        Simple proportional controller to drive a robot to a target point.
        
        Args:
            robot_name: Name of the robot
            target_x, target_y: Target position (m)
            k_linear, k_angular: Control gains
            distance_threshold: Goal tolerance (m)
            
        Returns:
            bool: True if goal reached
        """
        robot = self.robots[robot_name]
        
        dx = target_x - robot.x
        dy = target_y - robot.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < distance_threshold:
            self.stop(robot_name)
            return True
        
        desired_theta = math.atan2(dy, dx)
        theta_error = self.normalize_angle(desired_theta - robot.theta)
        
        v = min(k_linear * distance, 0.22)
        omega = max(min(k_angular * theta_error, 2.84), -2.84)
        
        self.set_velocity(robot_name, v, omega)
        return False

    def go_to_points(self, targets: Dict[str, Tuple[float, float]], **kwargs) -> Dict[str, bool]:
        """
        Drive multiple robots to their respective target points.
        
        Args:
            targets: Dict mapping robot_name -> (target_x, target_y)
            **kwargs: Additional arguments passed to go_to_point
            
        Returns:
            Dict mapping robot_name -> goal_reached (bool)
        """
        results = {}
        for robot_name, (tx, ty) in targets.items():
            results[robot_name] = self.go_to_point(robot_name, tx, ty, **kwargs)
        return results

    def print_states(self):
        """Print state of all robots."""
        for name in self.robot_names:
            robot = self.robots[name]
            if robot.odom_received:
                self.get_logger().info(
                    f'{name}: Pos({robot.x:.2f}, {robot.y:.2f}, {math.degrees(robot.theta):.1f}°) '
                    f'Vel({robot.vx:.2f}, {robot.vy:.2f}, {robot.omega:.2f})'
                )
            else:
                self.get_logger().info(f'{name}: No odometry data')


# ============================================
# MAIN FUNCTION
# ============================================

def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    # Create controller for 4 robots (tb3_1, tb3_2, tb3_3, tb3_4)
    controller = MultiUnicycleController()
    
    # Or specify custom robot names:
    # controller = MultiUnicycleController(['robot1', 'robot2', 'robot3', 'robot4'])
    
    try:
        controller.get_logger().info('Waiting for odometry data from all robots...')
        
        # Wait for odometry from at least some robots
        timeout_count = 0
        while controller.ready_count() < controller.num_robots and timeout_count < 100:
            rclpy.spin_once(controller, timeout_sec=0.1)
            timeout_count += 1
            if timeout_count % 10 == 0:
                controller.get_logger().info(
                    f'Odometry received from {controller.ready_count()}/{controller.num_robots} robots'
                )
        
        controller.get_logger().info(f'Controller ready with {controller.ready_count()} robots!')
        controller.print_states()
        
        # ============================================
        # USER CODE: Add your control logic here
        # ============================================
        
        # Example 1: Set different velocities for each robot
        # controller.set_velocity('tb3_1', 0.1, 0.0)    # Forward
        # controller.set_velocity('tb3_2', 0.1, 0.5)    # Forward + turn
        # controller.set_velocity('tb3_3', 0.0, 0.5)    # Rotate in place
        # controller.set_velocity('tb3_4', -0.1, 0.0)   # Backward
        
        # Example 2: Set velocities for all robots at once
        # controller.set_velocities({
        #     'tb3_1': (0.2, 0.0),
        #     'tb3_2': (0.2, 0.1),
        #     'tb3_3': (0.2, -0.1),
        #     'tb3_4': (0.2, 0.0)
        # })
        
        # Example 3: Go to points
        # targets = {
        #     'tb3_1': (1.0, 0.0),
        #     'tb3_2': (0.0, 1.0),
        #     'tb3_3': (-1.0, 0.0),
        #     'tb3_4': (0.0, -1.0)
        # }
        # while rclpy.ok():
        #     rclpy.spin_once(controller, timeout_sec=0.02)
        #     results = controller.go_to_points(targets)
        #     if all(results.values()):
        #         controller.get_logger().info('All robots reached goals!')
        #         break
        
        # Example 4: Direct state access
        # x1 = controller['tb3_1'].x
        # y1 = controller['tb3_1'].y
        # theta1 = controller['tb3_1'].theta
        
        # Example 5: Get all positions as numpy array (useful for vectorized control)
        # positions = controller.get_all_positions_array()  # Shape: (4, 3)
        # velocities = controller.get_all_velocities_global_array()  # Shape: (4, 3)
        
        # Default: Just spin
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down...')
        controller.stop_all()
    
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
