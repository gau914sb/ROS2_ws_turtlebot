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
from math import sin, cos, atan2, sqrt, pi, acos, asin, degrees, radians
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from typing import Dict, List, Tuple, Optional
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
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

        # Controller related initializations and variables

        self.pstar = np.array([[0.2], [0.2], [1.6], [0.2], [2.4], [2.4], [0.2], [1.6]])  # Desired formation points

        self.H = np.array([[-1, 1, 0, 0],    #Connection matrix for 4 robots
                      [0, -1, 1, 0],
                      [0, 0, -1, 1],
                      [1, 0, 0, -1],
                      [0, 1, 0, -1]])
        
        self.Hbar = np.kron(self.H, np.eye(2))

        self.v_max = 0.2  # Max linear velocity

        self.omega_max = 2.84  # Max angular velocity

        self.K_p_w = 1.0  # Angular velocity gain

        self.estar = self.Hbar @ self.pstar

        self.r = 0.2 # Agent radius
        self.Rs = 0.4  # Safety radius
        self.Rc = 3 # Connectivity radius

        self.Rc_bar = sqrt(1-(self.r/(self.Rc-self.r))**2)
        self.Rs_bar = sqrt(1-(self.r/(self.Rs+self.r))**2)
        self.theta_min = acos(self.Rc_bar)
        self.theta_max = acos(self.Rs_bar)

        self.theta_star = np.array([])
        self.gbar_star = np.array([])
        self.Kc = np.array([])
        self.Ks = np.array([])

        for i in range(0, 10, 2):

            temp1 = asin(self.r/np.linalg.norm(self.estar[i:i+2]))
            self.theta_star = np.append(self.theta_star, temp1)
            kc = self.Rc_bar**2 - (cos(temp1))**2
            self.Kc = np.append(self.Kc, kc)
            ks = (cos(temp1))**2 - self.Rs_bar**2
            self.Ks = np.append(self.Ks, ks)

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
        """Log current positions of all robots."""
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
            self.last_log_time = current_time

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
        # Log trajectories
        self.log_trajectories()
        
        # Skip control if not all robots have odometry
        if not self.is_ready():
            return
        
        p_internal = self.get_all_xy_array().flatten()  # Shape (8,)
        e = self.Hbar @ p_internal  # Shape (10,) - edge vectors

        g = np.array([])
        for i in range(0, 10, 2):
            e_vec = e[i:i+2]
            g1 = e_vec / np.linalg.norm(e_vec)
            g = np.append(g, g1)

        gbar = np.array([])
        E = np.array([])
        Zc  = np.array([])
        Zs  = np.array([])
        s = np.array([])
        e_bar = np.array([])
        T = np.array([])

        for i in range(0, 10, 2):
            e_vec = e[i:i+2]
            e_norm = np.linalg.norm(e_vec)
            theta = asin(self.r / e_norm)
            T = np.append(T, theta)
            dbar = sqrt(e_norm**2 - self.r**2)
            R_o = np.array([[cos(theta), -sin(theta)],
                            [sin(theta), cos(theta)]])
            g_vec = g[i:i+2]
            temp1 = R_o @ g_vec
            ebar = dbar * temp1
            e_bar = np.append(e_bar, ebar)
            gbar = np.append(gbar, temp1)
            c = int(i/2)
            E_temp = (cos(theta))**2 - (cos(self.theta_star[c]))**2

            if (E_temp > 0):
                s_temp = 1
            else:
                s_temp = 0
            
            s = np.append(s, s_temp)
            E = np.append(E, E_temp)
            zc = E_temp / self.Kc[c]
            Zc = np.append(Zc, zc)
            zs = E_temp / self.Ks[c]
            Zs = np.append(Zs, zs)

        phi = np.array([])
        D_g = None
        D_gbar = None
        D_Pg = None
        D_Pgbar = None

        for i in range(0, 5):
            z = (s[i] * Zc[i]) + ((1 - s[i]) * Zs[i])
            phi_temp = (z/(1-z**2)) * ((s[i]/self.Kc[i]) + ((1 - s[i])/self.Ks[i]))
            phi = np.append(phi, phi_temp)

        for i in range(0, 10, 2):
            g_vec = g[i:i+2].reshape(2, 1)
            gbar_vec = gbar[i:i+2].reshape(2, 1)
            ddot = (g_vec.T @ gbar_vec)[0, 0]  # scalar dot product
            
            block_g = ddot * g_vec
            block_gbar = ddot * gbar_vec
            pg = np.eye(2) - g_vec @ g_vec.T
            pg_bar = np.eye(2) - gbar_vec @ gbar_vec.T
            
            if D_g is None:
                D_g = block_g
                D_gbar = block_gbar
                D_Pg = pg
                D_Pgbar = pg_bar
            else:
                D_g = block_diag(D_g, block_g)
                D_gbar = block_diag(D_gbar, block_gbar)
                D_Pg = block_diag(D_Pg, pg)
                D_Pgbar = block_diag(D_Pgbar, pg_bar)

        d_dt = -2*(self.Hbar.T @ D_Pg @ D_gbar @ phi) - 2*(self.Hbar.T @ D_Pgbar @ D_g @ phi)

        #  Unicycle conversion starts here

        v_mag = np.array([np.linalg.norm(d_dt[i:i+2]) for i in range(0, 8, 2)])

        theta_desired = np.array([atan2(d_dt[i+1], d_dt[i]) for i in range(0, 8, 2)])

        theta_current = self.get_all_heading_array().flatten()

        theta_error = np.array([self.normalize_angle(theta_desired[i] - theta_current[i]) 
                        for i in range(self.num_robots)])
        
        v_uni = v_mag * np.cos(theta_error)

        w_uni = self.K_p_w * theta_error

        v_uni = np.clip(v_uni, -self.v_max, self.v_max)

        w_uni = np.clip(w_uni, -self.omega_max, self.omega_max)

        for i, name in enumerate(self.robot_names):
            self.set_velocity(name, v_uni[i], w_uni[i])

        # Example: Implement your parallel control law for all robots here
        # 
        # for name in self.robot_names:
        #     x, y, theta = self.get_position(name)
        #     # Compute control for each robot independently
        #     v, omega = self.compute_control(name, x, y, theta)
        #     self.set_velocity(name, v, omega)

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
    
    def plot_trajectories(self, save_path=None):
        """
        Plot trajectories of all robots with heading arrows.
        
        Args:
            save_path: Optional path to save the figure
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
            print(f'Figure saved to {save_path}')
        
        plt.show()
    
    def animate_trajectories(self, interval=50, save_path=None):
        """
        Create an animation of all robots moving.
        
        Args:
            interval: Animation interval in milliseconds
            save_path: Optional path to save animation (as .gif or .mp4)
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
        
        plt.show()


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
        
        # Plot trajectories after ROS cleanup
        total_points = sum(len(controller.robots[name].trajectory_x) for name in controller.robot_names)
        if total_points > 2:
            print(f'Recorded {total_points} total trajectory points')
            print('Generating plot...')
            
            # Static plot with all trajectories
            controller.plot_trajectories()
            
            # Uncomment for animation
            # controller.animate_trajectories(interval=50)


if __name__ == '__main__':
    main()
