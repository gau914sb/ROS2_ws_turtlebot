#!/usr/bin/env python3
"""
Multi-Robot Unicycle Controller Template for TurtleBot3
=======================================================
A clean template for controlling 4 TurtleBots independently using unicycle model.

Updated with Double-Angle (Visual Cone) Connectivity and Collision Avoidance.

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
from math import sin, cos, atan2, sqrt, pi, acos, asin, degrees, radians
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from typing import Dict, List, Tuple, Optional
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
from datetime import datetime
import os
import csv
from scipy.linalg import block_diag


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
        
        # Trajectory logging
        self.trajectory_x = []
        self.trajectory_y = []
        self.trajectory_theta = []


class MultiUnicycleController(Node):
    """
    Unicycle controller for multiple TurtleBot3 robots.
    
    Provides independent control for each robot with:
    - Unicycle velocity command interface (v, ω)
    - Global frame position feedback (x, y, θ)
    - Global frame velocity feedback (vx, vy, ω)
    """

    def __init__(self, robot_names: List[str] = None, namespace_prefix: str = 'bot'):
        """
        Initialize the multi-robot unicycle controller.
        
        Args:
            robot_names: List of robot names. If None, defaults to ['bot_0', 'bot_1', 'bot_2', 'bot_3']
            namespace_prefix: Prefix for auto-generated names (used if robot_names is None)
        """
        super().__init__('multi_unicycle_controller')
        
        # Default robot configuration
        if robot_names is None:
            self.num_robots = 4
            self.robot_names = [f'{namespace_prefix}_{i}' for i in range(self.num_robots)]
        else:
            self.robot_names = robot_names
            self.num_robots = len(robot_names)
        
        # ============================================
        # Robot State Storage
        # ============================================
        
        self.robots: Dict[str, RobotState] = {}
        for name in self.robot_names:
            self.robots[name] = RobotState(name)

        # Controller related initializations and variables

        # self.pstar = np.array([[0.6], [2.0], [1.8], [2.0], [2.4], [0.0], [0.0], [0.0]])  # Desired formation points
        # self.pstar = np.array([[0.15], [0.5], [0.45], [0.5], [0.6], [0.0], [0.0], [0.0]])  # Desired formation points
        # self.pstar = np.array([[0.30], [1.0], [0.90], [1.0], [1.2], [0.0], [0.0], [0.0]])  # Desired formation points
        self.pstar = np.array([[0.4], [0.3], [-0.4], [0.3], [-0.7], [-0.4], [0.7], [-0.4]])       

        # ============================================
        # DOUBLE-ANGLE MATHEMATICAL PARAMETERIZATION
        # ============================================
        self.H_Nset = np.array([[-1, 1, 0, 0],
                                [0, -1, 1, 0],
                                [0, 0, -1, 1],
                                [1, 0, 0, -1],
                                [0, 1, 0, -1]])
        self.Hbar_Nset = np.kron(self.H_Nset, np.eye(2))

        # Collision avoidance set
        self.H_Mset = np.array([[-1, 0, 1, 0]])
        self.Hbar_Mset = np.kron(self.H_Mset, np.eye(2))

        self.v_max = 0.055  # Max linear velocity
        self.omega_max = 0.13  # Max angular velocity
        self.K_p_w = 1.0  # Angular velocity gain
        self.length = 0.25

        self.estar = self.Hbar_Nset @ self.pstar

        # Radius parameters (Rcrit 7.5 scaled by 0.2 = 1.5)
        # self.r = 0.135 # Agent radius
        # self.Rs = 0.4  # Safety radius
        # self.Rc = 3.0 # Connectivity radius
        # self.Rcrit = 1.5 # Critical collision radius

        self.r = 0.135 # Agent radius
        self.Rs = 0.16  # Safety radius
        self.Rc = 1.6 # Connectivity radius
        self.Rcrit = 0.4 # Critical collision radius

        # Inter-agent center-to-center distance bounds (used for distance-error logging)
        self.dist_lower_bound = self.Rs + self.r  # Minimum allowed distance (collision/safety)
        self.dist_upper_bound = self.Rc - self.r  # Maximum allowed distance (connectivity)

        # Double angle bound formulation
        self.Rc_bar = 1 - (2 * (self.r / (self.Rc - self.r))**2)
        self.Rs_bar = 1 - (2 * (self.r / (self.Rs + self.r))**2)
        self.Rcrit_bar = 1 - (2 * (self.r / self.Rcrit)**2)

        self.psi_min = acos(self.Rc_bar)
        self.psi_max = acos(self.Rs_bar)
        self.psi_crit = acos(self.Rcrit_bar)

        self.psi_star = np.array([])
        self.Kc = np.array([])
        self.Ks = np.array([])

        for i in range(0, len(self.estar), 2):
            e_norm = np.linalg.norm(self.estar[i:i+2])
            temp1 = 2 * asin(self.r / e_norm)
            self.psi_star = np.append(self.psi_star, temp1)
            kc = self.Rc_bar**2 - (cos(temp1))**2
            self.Kc = np.append(self.Kc, kc)
            ks = (cos(temp1))**2 - self.Rs_bar**2
            self.Ks = np.append(self.Ks, ks)
            
        self.delta = self.Rs_bar**2 - self.Rcrit_bar**2
        
        # Maintain backwards compatibility for logging distances
        self.H = self.H_Nset
        self.Hbar = self.Hbar_Nset

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
        
        # ============================================
        # Trajectory Logging
        # ============================================
        
        self.trajectory_time = []
        self.start_time = None
        self.log_interval = 0.1  # Log every 0.1 seconds
        self.last_log_time = 0.0
        self.logging_enabled = True
        
        # Robot colors for plotting
        self.robot_colors = {
            self.robot_names[0]: 'blue',
            self.robot_names[1]: 'red',
            self.robot_names[2]: 'green',
            self.robot_names[3]: 'orange'
        } if self.num_robots >= 4 else {name: f'C{i}' for i, name in enumerate(self.robot_names)}
        
        # Distance logging for neighbor pairs
        self.distance_log = {i: [] for i in range(self.H.shape[0])}  # One list per edge
        self.error_log = {i: [] for i in range(self.H.shape[0])}     # Distance error (actual - desired) per edge
        
        # Calculate desired distances from pstar
        self.desired_distances = []
        for i in range(0, len(self.estar), 2):
            e_star_vec = self.estar[i:i+2].flatten()
            d_star = np.linalg.norm(e_star_vec)
            self.desired_distances.append(d_star)
        
        # Edge labels for plotting
        self.edge_labels = []
        pairs = self.get_connected_pairs()
        for i, j in pairs:
            self.edge_labels.append(f'{self.robot_names[i]} - {self.robot_names[j]}')

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
        robot.vx = robot.v_body * cos(robot.theta)
        robot.vy = robot.v_body * sin(robot.theta)
        
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
    
    def get_xy_array(self, robot_name: str) -> np.ndarray:
        """
        Get position as numpy array.
        
        Returns:
            np.ndarray: [x, y, theta]
        """
        robot = self.robots[robot_name]
        return np.array([robot.x, robot.y])
    
    def get_heading_array(self, robot_name: str) -> np.ndarray:
        """
        Get position as numpy array.
        
        Returns:
            np.ndarray: [x, y, theta]
        """
        robot = self.robots[robot_name]
        return np.array([robot.theta])

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
    
    def get_all_xy_array(self) -> np.ndarray:
        """
        Get all positions as a numpy array.
        
        Returns:
            np.ndarray: Shape (num_robots, 2) with [x, y] per row
        """
        return np.array([self.get_xy_array(name) for name in self.robot_names])
    
    def get_all_heading_array(self) -> np.ndarray:
        """
        Get all headings as a numpy array.
        
        Returns:
            np.ndarray: Shape (num_robots, 1) with [theta] per row
        """
        return np.array([self.get_heading_array(name) for name in self.robot_names])

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
        return atan2(siny_cosp, cosy_cosp)
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-π, π]."""
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def is_ready(self) -> bool:
        """Check if all robots have received odometry."""
        return all(robot.odom_received for robot in self.robots.values())
    
    def ready_count(self) -> int:
        """Get number of robots with odometry data."""
        return sum(1 for robot in self.robots.values() if robot.odom_received)

    def log_trajectories(self):
        """Log current positions of all robots and neighbor distances."""
        if not self.logging_enabled:
            return
            
        if self.start_time is None:
            self.start_time = time.time()
        
        current_time = time.time() - self.start_time
        if current_time - self.last_log_time >= self.log_interval:
            self.trajectory_time.append(current_time)
            for name in self.robot_names:
                robot = self.robots[name]
                if robot.odom_received:
                    robot.trajectory_x.append(robot.x)
                    robot.trajectory_y.append(robot.y)
                    robot.trajectory_theta.append(robot.theta)
            
            # Log neighbor distances and print distance errors
            if self.is_ready():
                positions = self.get_all_xy_array()  # Shape (num_robots, 2)
                pairs = self.get_connected_pairs()
                
                error_msgs = []
                for edge_idx, (i, j) in enumerate(pairs):
                    dist = np.linalg.norm(positions[i] - positions[j])
                    self.distance_log[edge_idx].append(dist)
                    
                    # Calculate distance error (current - desired)
                    error = dist - self.desired_distances[edge_idx]
                    self.error_log[edge_idx].append(error)
                    error_msgs.append(f"e{edge_idx+1}={error:+.3f}")
                
                # Log distance errors to terminal
                self.get_logger().info(f"t={current_time:.1f}s | Distance errors: {' | '.join(error_msgs)}")
            
            self.last_log_time = current_time

    # ============================================
    # CONTROL LOOP - DOUBLE ANGLE VISUAL CONE
    # ============================================
    def control_loop(self):
        """
        Main control loop - runs at control_rate Hz.
        """
        self.log_trajectories()
        
        if not self.is_ready():
            return
        
        p_internal = self.get_all_xy_array().flatten()  # Shape (8,)
        
        # Calculate Nset edges (Connectivity) and Mset edges (Collision Avoidance)
        e_N = self.Hbar_Nset @ p_internal
        e_M = self.Hbar_Mset @ p_internal

        m = self.Hbar_Nset.shape[0]     # 10 (5 edges x 2D)
        mhat = self.Hbar_Mset.shape[0]  # 2  (1 edge x 2D)

        R_psi_N = None
        xi = np.array([])
        E = np.array([])

        # ==================================
        # 1. Connectivity Preservation Loop
        # ==================================
        for i in range(0, m, 2):
            e_vec = e_N[i:i+2].reshape(2, 1)
            dij = np.linalg.norm(e_vec)

            # Prevent divide by zero if robots collide
            if dij < 1e-6: return

            asin_arg = float(np.clip(self.r / dij, -1.0, 1.0))
            psi_ij = 2 * asin(asin_arg)
            gij = e_vec / dij

            Rot_R = np.array([[cos(psi_ij/2), -sin(psi_ij/2)],
                              [sin(psi_ij/2),  cos(psi_ij/2)]])
            Rot_L = np.array([[cos(psi_ij/2),  sin(psi_ij/2)],
                              [-sin(psi_ij/2), cos(psi_ij/2)]])

            g_iRj = Rot_R @ gij
            g_iLj = Rot_L @ gij

            Pg_iRj = np.eye(2) - (g_iRj @ g_iRj.T)
            Pg_iLj = np.eye(2) - (g_iLj @ g_iLj.T)

            # f_gij results in a (2, 1) vector block
            f_gij = (Pg_iLj @ g_iRj) + (Pg_iRj @ g_iLj)
            f_psi_ij = 0.5 * sin(2 * psi_ij) / (1 + cos(psi_ij))

            block_N = f_psi_ij * f_gij
            if R_psi_N is None:
                R_psi_N = block_N
            else:
                R_psi_N = block_diag(R_psi_N, block_N)

            c = int(i/2)
            E_ij = (cos(psi_ij))**2 - (cos(self.psi_star[c]))**2

            q = 1.0 if E_ij > 0 else 0.0
            K = (q * self.Kc[c]) + ((1 - q) * self.Ks[c])

            denom = K**2 - E_ij**2
            xi_temp = 0.0 if abs(denom) < 1e-6 else 4 * E_ij / denom
            xi = np.append(xi, xi_temp)
            E = np.append(E, E_ij)

        # ==================================
        # 2. Collision Avoidance Loop
        # ==================================
        R_psi_M = None
        xi_beta = np.array([])

        for i in range(0, mhat, 2):
            e_vec = e_M[i:i+2].reshape(2, 1)
            dik = np.linalg.norm(e_vec)

            if dik < 1e-6: return

            asin_arg = float(np.clip(self.r / dik, -1.0, 1.0))
            psi_ik = 2 * asin(asin_arg)
            gik = e_vec / dik

            Rot_R = np.array([[cos(psi_ik/2), -sin(psi_ik/2)],
                              [sin(psi_ik/2),  cos(psi_ik/2)]])
            Rot_L = np.array([[cos(psi_ik/2),  sin(psi_ik/2)],
                              [-sin(psi_ik/2), cos(psi_ik/2)]])

            g_iRk = Rot_R @ gik
            g_iLk = Rot_L @ gik

            Pg_iRk = np.eye(2) - (g_iRk @ g_iRk.T)
            Pg_iLk = np.eye(2) - (g_iLk @ g_iLk.T)

            f_gik = (Pg_iLk @ g_iRk) + (Pg_iRk @ g_iLk)
            f_psi_ik = 0.5 * sin(2 * psi_ik) / (1 + cos(psi_ik))

            block_M = f_psi_ik * f_gik
            if R_psi_M is None:
                R_psi_M = block_M
            else:
                R_psi_M = block_diag(R_psi_M, block_M)

            if psi_ik > self.psi_crit:
                beta_ik = (cos(psi_ik))**2 - (cos(self.psi_crit))**2
            else:
                beta_ik = 0.0

            denom_beta = self.delta**2 - beta_ik**2
            xi_beta_temp = 0.0 if abs(denom_beta) < 1e-6 else 4 * beta_ik / denom_beta
            xi_beta = np.append(xi_beta, xi_beta_temp)

        # Reshape scalers for dot products
        xi = xi.reshape(-1, 1)
        xi_beta = xi_beta.reshape(-1, 1)

        # ==================================
        # 3. Choose your Control Law (d_dt)
        # ==================================
        
        # VARIANT 1: Connectivity Preservation Only
        # d_dt = - (self.Hbar_Nset.T @ R_psi_N @ xi)
        
        # VARIANT 2: Connectivity Preservation + Collision Avoidance 
        # Uncomment the line below to activate collision avoidance
        d_dt = - 10000000 * (self.Hbar_Nset.T @ R_psi_N @ xi + self.Hbar_Mset.T @ R_psi_M @ xi_beta)

        # Variant 3
        # d_dt = -10000000 * (self.Hbar_Nset.T @ R_psi_N @ E)

        d_dt = d_dt.flatten() # Flatten to 1D array for Unicycle processing

        # ==================================
        # 4. Unicycle Conversion
        # ==================================

        theta_current = self.get_all_heading_array().flatten()

        v_uni = np.array([0.0]*self.num_robots)
        w_uni = np.array([0.0]*self.num_robots)

        for i in range(self.num_robots):
            v_uni[i] = 2 * (d_dt[2*i] * cos(theta_current[i]) + d_dt[2*i+1] * sin(theta_current[i]))

        for i in range(self.num_robots):
            w_uni[i] = -d_dt[2*i] * sin(theta_current[i]) / self.length + d_dt[2*i+1] * cos(theta_current[i]) / self.length

        v_uni = np.clip(v_uni, -self.v_max, self.v_max)
        w_uni = np.clip(w_uni, -self.omega_max, self.omega_max)

        for i, name in enumerate(self.robot_names):
            self.set_velocity(name, v_uni[i], w_uni[i])


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
        distance = sqrt(dx*dx + dy*dy)
        
        if distance < distance_threshold:
            self.stop(robot_name)
            return True
        
        desired_theta = atan2(dy, dx)
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
                    f'{name}: Pos({robot.x:.2f}, {robot.y:.2f}, {degrees(robot.theta):.1f}°) '
                    f'Vel({robot.vx:.2f}, {robot.vy:.2f}, {robot.omega:.2f})'
                )
            else:
                self.get_logger().info(f'{name}: No odometry data')

    # ============================================
    # PLOTTING AND ANIMATION
    # ============================================
    
    def get_save_directory(self, timestamp: str = None) -> str:
        """
        Get or create the directory for saving plots.
        Creates a timestamped subfolder for better organization.
        
        Args:
            timestamp: Optional timestamp string. If None, uses current time.
        
        Returns:
            str: Path to the save directory
        """
        if timestamp is None:
            timestamp = self.get_timestamp()
        
        base_dir = os.path.expanduser('~/turtlebot3_simulations/plots')
        save_dir = os.path.join(base_dir, timestamp)
        os.makedirs(save_dir, exist_ok=True)
        return save_dir
    
    def get_timestamp(self) -> str:
        """
        Get current timestamp string for filenames.
        
        Returns:
            str: Timestamp in format YYYY-MM-DD_HH-MM-SS
        """
        return datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    
    def save_all_plots(self):
        """
        Save trajectory/distance PNG plots and the full logged data as CSV, with timestamp.
        Creates a subfolder with the timestamp for organization.
        Called automatically on shutdown.

        Note: GIF/animation export was removed (it was slow and RAM-heavy for long runs).
        Use the CSV output with plot_from_csv-style post-processing instead if an
        animation is ever needed.
        """
        timestamp = self.get_timestamp()
        save_dir = self.get_save_directory(timestamp)

        print(f'Saving outputs to {save_dir}...')

        # Save trajectory plot (PNG)
        traj_path = os.path.join(save_dir, 'trajectory.png')
        self.plot_trajectories(save_path=traj_path, show=False)

        # Save distances plot (PNG)
        dist_path = os.path.join(save_dir, 'distances.png')
        self.plot_distances(save_path=dist_path, show=False)

        # Save full logged data (positions, headings, distances, errors, bounds) as CSV
        csv_path = os.path.join(save_dir, 'log_data.csv')
        self.save_csv(save_path=csv_path)

        print(f'All outputs saved successfully to {save_dir}!')

    def save_csv(self, save_path: str):
        """
        Save all logged data to a single CSV file:
          - x, y, theta (global frame) for every robot
          - all 5 inter-robot distances (d1..d5)
          - all 5 distance errors (e1..e5), where error = actual - desired distance
          - the lower bound (Rs + r) and upper bound (Rc - r) on inter-robot distance

        Args:
            save_path: Full path to the output .csv file
        """
        num_edges = self.H.shape[0]

        # Rows are limited by the shortest logged series, since trajectory/distance
        # logging can start at slightly different ticks (distances need all robots ready).
        lengths = [len(self.trajectory_time)]
        for name in self.robot_names:
            lengths.append(len(self.robots[name].trajectory_x))
        for edge_idx in range(num_edges):
            lengths.append(len(self.distance_log[edge_idx]))
            lengths.append(len(self.error_log[edge_idx]))

        num_rows = min(lengths) if lengths else 0

        if num_rows == 0:
            print('No logged data available to save to CSV')
            return

        header = ['time']
        for name in self.robot_names:
            header += [f'{name}_x', f'{name}_y', f'{name}_theta']
        for edge_idx in range(num_edges):
            header.append(f'd{edge_idx+1}')
        for edge_idx in range(num_edges):
            header.append(f'e{edge_idx+1}')
        header += ['lower_bound', 'upper_bound']

        with open(save_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)

            for k in range(num_rows):
                row = [self.trajectory_time[k]]
                for name in self.robot_names:
                    robot = self.robots[name]
                    row += [robot.trajectory_x[k], robot.trajectory_y[k], robot.trajectory_theta[k]]
                for edge_idx in range(num_edges):
                    row.append(self.distance_log[edge_idx][k])
                for edge_idx in range(num_edges):
                    row.append(self.error_log[edge_idx][k])
                row += [self.dist_lower_bound, self.dist_upper_bound]
                writer.writerow(row)

        print(f'Logged data ({num_rows} rows) saved to {save_path}')
    
    def get_connected_pairs(self) -> List[Tuple[int, int]]:
        """
        Get list of connected robot pairs from the incidence matrix H.
        
        Returns:
            List of tuples (i, j) where robots i and j are connected
        """
        pairs = []
        for edge_idx in range(self.H.shape[0]):
            edge = self.H[edge_idx, :]
            # Find the two robots connected by this edge (-1 and +1 in the row)
            robot_indices = np.where(edge != 0)[0]
            if len(robot_indices) == 2:
                pairs.append((robot_indices[0], robot_indices[1]))
        return pairs
    
    def plot_neighbor_connections(self, ax, positions: np.ndarray, linestyle: str = '-', 
                                   linewidth: float = 1.5, alpha: float = 0.7, 
                                   color: str = 'gray', label: str = None):
        """
        Plot connections between neighboring robots.
        
        Args:
            ax: Matplotlib axis
            positions: Array of shape (num_robots, 2) with [x, y] positions
            linestyle: Line style ('-' for solid, '--' for dashed, ':' for dotted)
            linewidth: Width of connection lines
            alpha: Transparency
            color: Line color
            label: Optional label for legend
        """
        pairs = self.get_connected_pairs()
        for idx, (i, j) in enumerate(pairs):
            x_vals = [positions[i, 0], positions[j, 0]]
            y_vals = [positions[i, 1], positions[j, 1]]
            if idx == 0 and label:
                ax.plot(x_vals, y_vals, linestyle=linestyle, linewidth=linewidth, 
                       alpha=alpha, color=color, label=label)
            else:
                ax.plot(x_vals, y_vals, linestyle=linestyle, linewidth=linewidth, 
                       alpha=alpha, color=color)
    
    def plot_trajectories(self, save_path=None, show=True):
        """
        Plot trajectories of all robots.
        
        Args:
            save_path: Optional path to save the figure
            show: Whether to display the plot (default True)
        """
        # Check if we have data
        has_data = False
        for name in self.robot_names:
            if len(self.robots[name].trajectory_x) > 1:
                has_data = True
                break
        
        if not has_data:
            print('Not enough trajectory data to plot')
            return
        
        fig, ax = plt.subplots(figsize=(12, 12))
        
        # Plot each robot's trajectory
        for name in self.robot_names:
            robot = self.robots[name]
            if len(robot.trajectory_x) < 2:
                continue
            
            color = self.robot_colors.get(name, 'blue')
            
            # Plot trajectory
            ax.plot(robot.trajectory_x, robot.trajectory_y, '-', 
                   color=color, linewidth=1.5, label=f'{name} trajectory')
            
            # Plot start and end points
            ax.plot(robot.trajectory_x[0], robot.trajectory_y[0], 'o', 
                   color=color, markersize=10, markeredgecolor='black', markeredgewidth=1.5)
            ax.plot(robot.trajectory_x[-1], robot.trajectory_y[-1], 's', 
                   color=color, markersize=10, markeredgecolor='black', markeredgewidth=1.5)
        
        # Plot neighbor connections at initial positions (dotted)
        initial_positions = np.array([[self.robots[name].trajectory_x[0], 
                                        self.robots[name].trajectory_y[0]] 
                                       for name in self.robot_names 
                                       if len(self.robots[name].trajectory_x) > 0])
        if len(initial_positions) == self.num_robots:
            self.plot_neighbor_connections(ax, initial_positions, linestyle=':', 
                                           linewidth=2.0, alpha=0.6, color='gray',
                                           label='Initial connections')
        
        # Plot neighbor connections at final positions (solid)
        final_positions = np.array([[self.robots[name].trajectory_x[-1], 
                                     self.robots[name].trajectory_y[-1]] 
                                    for name in self.robot_names 
                                    if len(self.robots[name].trajectory_x) > 0])
        if len(final_positions) == self.num_robots:
            self.plot_neighbor_connections(ax, final_positions, linestyle='-', 
                                           linewidth=2.5, alpha=0.8, color='black',
                                           label='Final connections')
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('Multi-Robot Trajectories', fontsize=14)
        ax.legend(loc='best', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        ax.set_aspect('equal')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f'Trajectory plot saved to {save_path}')
        
        if show:
            plt.show()
        else:
            plt.close(fig)
    
    def animate_trajectories(self, interval=50, save_path=None, show=True):
        """
        Create an animation of all robots moving.
        
        Args:
            interval: Animation interval in milliseconds
            save_path: Optional path to save animation (as .gif or .mp4)
            show: Whether to display the animation (default True)
        """
        # Check if we have data
        max_points = 0
        for name in self.robot_names:
            max_points = max(max_points, len(self.robots[name].trajectory_x))
        
        if max_points < 2:
            print('Not enough trajectory data to animate')
            return
        
        fig, ax = plt.subplots(figsize=(12, 12))
        
        # Compute axis limits from all trajectory data
        all_x, all_y = [], []
        for name in self.robot_names:
            robot = self.robots[name]
            all_x.extend(robot.trajectory_x)
            all_y.extend(robot.trajectory_y)
        
        if not all_x:
            print('No trajectory data')
            return
        
        margin = 0.5
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        
        # Initialize plot elements for each robot
        trajectory_lines = {}
        robot_markers = {}
        heading_arrows = {}
        
        for name in self.robot_names:
            color = self.robot_colors.get(name, 'blue')
            trajectory_lines[name], = ax.plot([], [], '-', color=color, 
                                               linewidth=1.5, alpha=0.7, label=name)
            robot_markers[name], = ax.plot([], [], 'o', color=color, markersize=10,
                                            markeredgecolor='black', markeredgewidth=1.5)
            heading_arrows[name] = ax.arrow(0, 0, 0, 0, head_width=0.06, head_length=0.04,
                                            fc=color, ec=color)
        
        # Plot initial neighbor connections (dotted) - static throughout animation
        initial_positions = np.array([[self.robots[name].trajectory_x[0], 
                                        self.robots[name].trajectory_y[0]] 
                                       for name in self.robot_names 
                                       if len(self.robots[name].trajectory_x) > 0])
        if len(initial_positions) == self.num_robots:
            self.plot_neighbor_connections(ax, initial_positions, linestyle=':', 
                                           linewidth=2.0, alpha=0.5, color='gray',
                                           label='Initial connections')
        
        # Initialize connection lines (solid) - updated every frame to show current distances
        connection_lines = []
        pairs = self.get_connected_pairs()
        for _ in pairs:
            line, = ax.plot([], [], '-', linewidth=2.5, alpha=0.8, color='black')
            connection_lines.append(line)
        
        # Time text
        time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, fontsize=12,
                           verticalalignment='top', fontfamily='monospace',
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('Multi-Robot Animation', fontsize=14)
        ax.legend(loc='upper right', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        def init():
            for name in self.robot_names:
                trajectory_lines[name].set_data([], [])
                robot_markers[name].set_data([], [])
            for line in connection_lines:
                line.set_data([], [])
            time_text.set_text('')
            return list(trajectory_lines.values()) + list(robot_markers.values()) + connection_lines + [time_text]
        
        def update(frame):
            for name in self.robot_names:
                robot = self.robots[name]
                color = self.robot_colors.get(name, 'blue')
                
                if frame < len(robot.trajectory_x):
                    # Update trajectory
                    trajectory_lines[name].set_data(robot.trajectory_x[:frame+1], 
                                                    robot.trajectory_y[:frame+1])
                    
                    # Update robot position
                    robot_markers[name].set_data([robot.trajectory_x[frame]], 
                                                  [robot.trajectory_y[frame]])
                    
                    # Update heading arrow
                    heading_arrows[name].remove()
                    arrow_length = 0.15
                    dx = arrow_length * np.cos(robot.trajectory_theta[frame])
                    dy = arrow_length * np.sin(robot.trajectory_theta[frame])
                    heading_arrows[name] = ax.arrow(robot.trajectory_x[frame], 
                                                    robot.trajectory_y[frame],
                                                    dx, dy, head_width=0.06, head_length=0.04,
                                                    fc=color, ec=color)
            
            # Update time text
            if frame < len(self.trajectory_time):
                time_text.set_text(f't = {self.trajectory_time[frame]:.1f}s')
            
            # Update solid connection lines at current frame positions (shows changing distances)
            current_positions = []
            for name in self.robot_names:
                robot = self.robots[name]
                if frame < len(robot.trajectory_x):
                    current_positions.append([robot.trajectory_x[frame], robot.trajectory_y[frame]])
                else:
                    current_positions.append([robot.trajectory_x[-1], robot.trajectory_y[-1]])
            current_positions = np.array(current_positions)
            
            # Update connection lines to show current inter-robot distances
            for idx, (i, j) in enumerate(pairs):
                x_vals = [current_positions[i, 0], current_positions[j, 0]]
                y_vals = [current_positions[i, 1], current_positions[j, 1]]
                connection_lines[idx].set_data(x_vals, y_vals)
            
            return list(trajectory_lines.values()) + list(robot_markers.values()) + connection_lines + [time_text]
        
        anim = FuncAnimation(fig, update, frames=max_points,
                            init_func=init, blit=False, interval=interval, repeat=True)
        
        if save_path:
            if save_path.endswith('.gif'):
                anim.save(save_path, writer='pillow', fps=1000//interval)
            else:
                anim.save(save_path, writer='ffmpeg', fps=1000//interval)
            print(f'Animation saved to {save_path}')
        
        if show:
            plt.show()
        else:
            plt.close(fig)
    
    def plot_distances(self, save_path=None, show=True):
        """
        Plot neighbor distances over time for all edges (Figure 2).
        Shows actual distances vs desired distances.
        
        Args:
            save_path: Optional path to save the figure
            show: Whether to display the plot (default True)
        """
        # Check if we have distance data
        has_data = any(len(self.distance_log[i]) > 1 for i in range(self.H.shape[0]))
        
        if not has_data:
            print('Not enough distance data to plot')
            return
        
        fig, ax = plt.subplots(figsize=(12, 8))
        
        # Colors for each edge
        edge_colors = ['blue', 'red', 'green', 'orange', 'purple']
        
        # Get the time array (trimmed to match distance data length)
        num_points = len(self.distance_log[0])
        time_data = self.trajectory_time[:num_points]
        
        # Plot actual distances for each edge
        pairs = self.get_connected_pairs()
        for edge_idx in range(self.H.shape[0]):
            if len(self.distance_log[edge_idx]) > 1:
                color = edge_colors[edge_idx % len(edge_colors)]
                i, j = pairs[edge_idx]
                label = f'Edge {edge_idx+1}: {self.robot_names[i]}-{self.robot_names[j]}'
                
                # Plot actual distance (solid line)
                ax.plot(time_data, self.distance_log[edge_idx], '-', 
                       color=color, linewidth=2, label=f'{label} (actual)')
                
                # Plot desired distance (dashed horizontal line)
                ax.axhline(y=self.desired_distances[edge_idx], color=color, 
                          linestyle='--', linewidth=1.5, alpha=0.7,
                          label=f'{label} (desired: {self.desired_distances[edge_idx]:.3f}m)')
        
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Distance (m)', fontsize=12)
        ax.set_title('Figure 2: Neighbor Distances Over Time', fontsize=14)
        ax.legend(loc='best', fontsize=9, ncol=2)
        ax.grid(True, alpha=0.3)
        
        # Add lower/upper inter-robot distance bound lines
        ax.axhline(y=self.dist_lower_bound, color='red', linestyle=':', linewidth=1, alpha=0.5,
                  label=f'Lower bound (Rs+r): {self.dist_lower_bound}m')
        ax.axhline(y=self.dist_upper_bound, color='gray', linestyle=':', linewidth=1, alpha=0.5,
                  label=f'Upper bound (Rc-r): {self.dist_upper_bound}m')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f'Distances plot saved to {save_path}')
        
        if show:
            plt.show()
        else:
            plt.close(fig)
    
    def animate_combined(self, interval=50, save_path=None, show=True):
        """
        Create a combined animation with trajectory and distance plots side by side.
        Layout: 1 row x 2 columns (trajectory on left, distances on right)
        
        Args:
            interval: Animation interval in milliseconds
            save_path: Optional path to save animation (as .gif or .mp4)
            show: Whether to display the animation (default True)
        """
        # Check if we have data
        max_points = 0
        for name in self.robot_names:
            max_points = max(max_points, len(self.robots[name].trajectory_x))
        
        if max_points < 2:
            print('Not enough trajectory data to animate')
            return
        
        has_distance_data = any(len(self.distance_log[i]) > 1 for i in range(self.H.shape[0]))
        if not has_distance_data:
            print('Not enough distance data to animate')
            return
        
        # Create figure with 1 row x 2 columns
        fig, (ax_traj, ax_dist) = plt.subplots(1, 2, figsize=(20, 9))
        
        # ============================================
        # LEFT PLOT: Trajectory Animation
        # ============================================
        
        # Compute axis limits from all trajectory data
        all_x, all_y = [], []
        for name in self.robot_names:
            robot = self.robots[name]
            all_x.extend(robot.trajectory_x)
            all_y.extend(robot.trajectory_y)
        
        if not all_x:
            print('No trajectory data')
            return
        
        margin = 0.5
        ax_traj.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax_traj.set_ylim(min(all_y) - margin, max(all_y) + margin)
        
        # Initialize plot elements for each robot
        trajectory_lines = {}
        robot_markers = {}
        heading_arrows = {}
        
        for name in self.robot_names:
            color = self.robot_colors.get(name, 'blue')
            trajectory_lines[name], = ax_traj.plot([], [], '-', color=color, 
                                                    linewidth=1.5, alpha=0.7, label=name)
            robot_markers[name], = ax_traj.plot([], [], 'o', color=color, markersize=10,
                                                 markeredgecolor='black', markeredgewidth=1.5)
            heading_arrows[name] = ax_traj.arrow(0, 0, 0, 0, head_width=0.06, head_length=0.04,
                                                  fc=color, ec=color)
        
        # Plot initial neighbor connections (dotted) - static throughout animation
        initial_positions = np.array([[self.robots[name].trajectory_x[0], 
                                        self.robots[name].trajectory_y[0]] 
                                       for name in self.robot_names 
                                       if len(self.robots[name].trajectory_x) > 0])
        if len(initial_positions) == self.num_robots:
            pairs = self.get_connected_pairs()
            for idx, (i, j) in enumerate(pairs):
                x_vals = [initial_positions[i, 0], initial_positions[j, 0]]
                y_vals = [initial_positions[i, 1], initial_positions[j, 1]]
                if idx == 0:
                    ax_traj.plot(x_vals, y_vals, ':', linewidth=2.0, alpha=0.5, 
                                color='gray', label='Initial connections')
                else:
                    ax_traj.plot(x_vals, y_vals, ':', linewidth=2.0, alpha=0.5, color='gray')
        
        # Initialize connection lines (solid) - updated every frame
        connection_lines = []
        pairs = self.get_connected_pairs()
        for _ in pairs:
            line, = ax_traj.plot([], [], '-', linewidth=2.5, alpha=0.8, color='black')
            connection_lines.append(line)
        
        # Time text on trajectory plot
        time_text = ax_traj.text(0.02, 0.98, '', transform=ax_traj.transAxes, fontsize=12,
                                  verticalalignment='top', fontfamily='monospace',
                                  bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        ax_traj.set_xlabel('X (m)', fontsize=12)
        ax_traj.set_ylabel('Y (m)', fontsize=12)
        ax_traj.set_title('Robot Trajectories', fontsize=14)
        ax_traj.legend(loc='upper right', fontsize=9)
        ax_traj.grid(True, alpha=0.3)
        ax_traj.set_aspect('equal')
        
        # ============================================
        # RIGHT PLOT: Distance Animation
        # ============================================
        
        edge_colors = ['blue', 'red', 'green', 'orange', 'purple']
        
        # Get time and distance data
        num_dist_points = len(self.distance_log[0])
        time_data = self.trajectory_time[:num_dist_points]
        
        # Set up distance plot axes
        max_time = max(time_data) if time_data else 1.0
        all_distances = []
        for edge_idx in range(self.H.shape[0]):
            all_distances.extend(self.distance_log[edge_idx])
        
        ax_dist.set_xlim(0, max_time * 1.05)
        ax_dist.set_ylim(0, max(all_distances + self.desired_distances) * 1.1)
        
        # Plot desired distances (static dashed lines)
        for edge_idx in range(self.H.shape[0]):
            color = edge_colors[edge_idx % len(edge_colors)]
            i, j = pairs[edge_idx]
            ax_dist.axhline(y=self.desired_distances[edge_idx], color=color, 
                           linestyle='--', linewidth=1.5, alpha=0.5)
        
        # Initialize distance lines (animated)
        distance_lines = []
        for edge_idx in range(self.H.shape[0]):
            color = edge_colors[edge_idx % len(edge_colors)]
            i, j = pairs[edge_idx]
            label = f'{self.robot_names[i]}-{self.robot_names[j]}'
            line, = ax_dist.plot([], [], '-', color=color, linewidth=2, label=label)
            distance_lines.append(line)
        
        # Vertical time indicator line
        time_indicator, = ax_dist.plot([], [], 'k-', linewidth=1.5, alpha=0.7)
        
        ax_dist.set_xlabel('Time (s)', fontsize=12)
        ax_dist.set_ylabel('Distance (m)', fontsize=12)
        ax_dist.set_title('Neighbor Distances Over Time', fontsize=14)
        ax_dist.legend(loc='upper right', fontsize=9)
        ax_dist.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        def init():
            # Initialize trajectory elements
            for name in self.robot_names:
                trajectory_lines[name].set_data([], [])
                robot_markers[name].set_data([], [])
            for line in connection_lines:
                line.set_data([], [])
            time_text.set_text('')
            
            # Initialize distance elements
            for line in distance_lines:
                line.set_data([], [])
            time_indicator.set_data([], [])
            
            return (list(trajectory_lines.values()) + list(robot_markers.values()) + 
                    connection_lines + [time_text] + distance_lines + [time_indicator])
        
        def update(frame):
            # ============================================
            # Update Trajectory Plot (left)
            # ============================================
            for name in self.robot_names:
                robot = self.robots[name]
                color = self.robot_colors.get(name, 'blue')
                
                if frame < len(robot.trajectory_x):
                    # Update trajectory
                    trajectory_lines[name].set_data(robot.trajectory_x[:frame+1], 
                                                    robot.trajectory_y[:frame+1])
                    
                    # Update robot position
                    robot_markers[name].set_data([robot.trajectory_x[frame]], 
                                                  [robot.trajectory_y[frame]])
                    
                    # Update heading arrow
                    heading_arrows[name].remove()
                    arrow_length = 0.15
                    dx = arrow_length * np.cos(robot.trajectory_theta[frame])
                    dy = arrow_length * np.sin(robot.trajectory_theta[frame])
                    heading_arrows[name] = ax_traj.arrow(robot.trajectory_x[frame], 
                                                          robot.trajectory_y[frame],
                                                          dx, dy, head_width=0.06, head_length=0.04,
                                                          fc=color, ec=color)
            
            # Update time text
            if frame < len(self.trajectory_time):
                time_text.set_text(f't = {self.trajectory_time[frame]:.1f}s')
            
            # Update connection lines
            current_positions = []
            for name in self.robot_names:
                robot = self.robots[name]
                if frame < len(robot.trajectory_x):
                    current_positions.append([robot.trajectory_x[frame], robot.trajectory_y[frame]])
                else:
                    current_positions.append([robot.trajectory_x[-1], robot.trajectory_y[-1]])
            current_positions = np.array(current_positions)
            
            for idx, (i, j) in enumerate(pairs):
                x_vals = [current_positions[i, 0], current_positions[j, 0]]
                y_vals = [current_positions[i, 1], current_positions[j, 1]]
                connection_lines[idx].set_data(x_vals, y_vals)
            
            # ============================================
            # Update Distance Plot (right)
            # ============================================
            if frame < num_dist_points:
                current_time = time_data[frame]
                
                # Update distance lines up to current frame
                for edge_idx in range(self.H.shape[0]):
                    distance_lines[edge_idx].set_data(time_data[:frame+1], 
                                                       self.distance_log[edge_idx][:frame+1])
                
                # Update time indicator vertical line
                time_indicator.set_data([current_time, current_time], [0, ax_dist.get_ylim()[1]])
            
            return (list(trajectory_lines.values()) + list(robot_markers.values()) + 
                    connection_lines + [time_text] + distance_lines + [time_indicator])
        
        anim = FuncAnimation(fig, update, frames=max_points,
                            init_func=init, blit=False, interval=interval, repeat=True)
        
        if save_path:
            if save_path.endswith('.gif'):
                anim.save(save_path, writer='pillow', fps=1000//interval)
            else:
                anim.save(save_path, writer='ffmpeg', fps=1000//interval)
            print(f'Combined animation saved to {save_path}')
        
        if show:
            plt.show()
        else:
            plt.close(fig)


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
        print('\nShutting down...')
    
    finally:
        # Stop all robots
        try:
            controller.stop_all()
        except:
            pass
        
        # Destroy node and shutdown
        try:
            controller.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass
        
        # Save all plots after ROS cleanup
        total_points = sum(len(controller.robots[name].trajectory_x) for name in controller.robot_names)
        if total_points > 2:
            print(f'Recorded {total_points} total trajectory points')
            controller.save_all_plots()


if __name__ == '__main__':
    main()