#!/usr/bin/env python3
"""
Circumnavigation Controller for TurtleBot3
===========================================
Implementation of the circumnavigation control law from MATLAB.

Target: Orbit around a fixed target point at desired radius rd.

Author: Based on circum_experimental_multiple_entry.m
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import time


class CircumnavigationController(Node):
    """
    Circumnavigation controller for a single TurtleBot3.
    
    Implements the control law to orbit around a target point.
    """

    def __init__(self, robot_namespace: str = 'tb3_1'):
        """
        Initialize the circumnavigation controller.
        
        Args:
            robot_namespace: Robot namespace (e.g., 'tb3_1' for multi-robot, 
                           empty string '' for single robot)
        """
        super().__init__('circumnavigation_controller')
        
        # Robot namespace configuration
        self.namespace = robot_namespace
        prefix = f'/{robot_namespace}' if robot_namespace else ''
        
        # ============================================
        # Robot State Variables (Global Frame)
        # ============================================
        
        # Position in global frame (from odometry)
        self.x = 0.0          # x position (m)
        self.y = 0.0          # y position (m)
        self.psi = 0.0        # heading/yaw angle (rad) - called psi to match MATLAB
        
        # Data validity flag
        self.odom_received = False
        
        # ============================================
        # Target Position
        # ============================================
        
        self.xt = 0.0         # target x position (m)
        self.yt = 0.0         # target y position (m)
        
        # ============================================
        # Controller Parameters (from MATLAB)
        # ============================================
        
        # Radii
        self.rd = 1.0         # desired orbit radius (m)
        self.ra = 0.7         # avoidance radius (m)
        self.rs = 0.4         # standoff radius (m)
        
        # Gains
        self.k = 1.0 / math.sqrt(self.rd**2 - self.ra**2)  # gain
        self.kappa = 0.5      # single entry gain
        self.delta = 0.5      # delta parameter
        
        # Velocity
        self.V = 0.15         # linear velocity (m/s) - reduced for TurtleBot3
        
        # Derived parameters
        self.beta = math.sqrt(self.rd**2 - self.ra**2) / self.ra
        
        # ============================================
        # Internal state for control law
        # ============================================
        
        self.theta = 0.0      # angle from radial to velocity (rad)
        self.r = 0.0          # distance to target (m)
        self.phi = 0.0        # angle from robot to target (rad)
        
        # Store for logging/debugging
        self.omega = 0.0
        self.h = 0.0
        self.eta = 0.0
        self.W = 0.0
        
        # ============================================
        # Trajectory Logging for Plotting
        # ============================================
        
        self.trajectory_x = []
        self.trajectory_y = []
        self.trajectory_psi = []
        self.trajectory_theta = []
        self.trajectory_r = []
        self.trajectory_omega = []
        self.trajectory_time = []
        self.start_time = None
        self.log_interval = 0.1  # Log every 0.1 seconds
        self.last_log_time = 0.0
        
        # ============================================
        # ROS2 Publishers and Subscribers
        # ============================================
        
        qos = QoSProfile(depth=10)
        
        # Velocity command publisher
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
        # Control Loop Timer
        # ============================================
        
        self.dt = 0.02        # 50 Hz control loop
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        # Status print timer (every 1 second)
        self.status_timer = self.create_timer(1.0, self.print_state)
        
        self.get_logger().info(f'Circumnavigation Controller initialized')
        self.get_logger().info(f'  Target: ({self.xt}, {self.yt})')
        self.get_logger().info(f'  Desired radius rd: {self.rd} m')
        self.get_logger().info(f'  Avoidance radius ra: {self.ra} m')
        self.get_logger().info(f'  Velocity V: {self.V} m/s')

    # ============================================
    # ODOMETRY CALLBACK
    # ============================================
    
    def odom_callback(self, msg: Odometry):
        """Process odometry message and update robot state."""
        # Extract position (global frame)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extract heading (quaternion to yaw)
        q = msg.pose.pose.orientation
        self.psi = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        
        self.odom_received = True

    # ============================================
    # CONTROL LOOP - Circumnavigation Law
    # ============================================
    
    def control_loop(self):
        """
        Main control loop implementing the circumnavigation control law.
        
        From MATLAB: circum_experimental_multiple_entry.m
        """
        if not self.odom_received:
            return
        
        # ----------------------------------------
        # Compute polar coordinates
        # ----------------------------------------
        
        # phi: angle from robot to target
        self.phi = math.atan2(self.yt - self.y, self.xt - self.x)
        
        # r: distance to target
        self.r = math.sqrt((self.x - self.xt)**2 + (self.y - self.yt)**2)
        
        # theta: angle from radial direction to velocity direction
        # psi = theta + phi  =>  theta = psi - phi
        self.theta = self.psi - self.phi
        
        # Normalize theta to [-pi, pi]
        self.theta = self.normalize_angle(self.theta)
        
        # ----------------------------------------
        # Control Law
        # ----------------------------------------
        
        # r_dot = -V * cos(theta)
        r_dot = -self.V * math.cos(self.theta)
        
        if self.r >= self.ra:
            # Compute h (Lyapunov-like function)
            sqrt_r2_ra2 = math.sqrt(self.r**2 - self.ra**2)
            sqrt_rd2_ra2 = math.sqrt(self.rd**2 - self.ra**2)
            
            self.h = (self.k * (sqrt_r2_ra2 
                      - self.ra * math.atan(sqrt_r2_ra2 / self.ra)
                      - sqrt_rd2_ra2 
                      + self.ra * math.atan(sqrt_rd2_ra2 / self.ra)) 
                      - math.log(self.r / self.rd))
            
            # eta and denominator
            self.eta = 1.0 - math.sin(self.theta) + self.h
            den = self.delta**2 - self.eta**2
            
            # Prevent division by zero
            if abs(den) < 1e-6:
                den = 1e-6 if den >= 0 else -1e-6
            
            # Omega (angular velocity control)
            # omega = -k*V*cos(asin(ra/r)) + (V/r)*(1-sin(theta)) - kappa*(r_dot/(V*den))
            
            # cos(asin(ra/r)) = sqrt(1 - (ra/r)^2) = sqrt(r^2 - ra^2)/r
            cos_asin_term = sqrt_r2_ra2 / self.r
            
            self.omega = (-self.k * self.V * cos_asin_term 
                         + (self.V / self.r) * (1.0 - math.sin(self.theta))
                         - self.kappa * (r_dot / (self.V * den)))
            
            # W (Lyapunov function value for monitoring)
            if den > 0:
                self.W = 0.5 * math.log(self.delta**2 / den)
            else:
                self.W = 0.0
        else:
            # Inside avoidance radius - stop rotation
            self.omega = 0.0
            self.h = 0.0
            self.eta = 0.0
            self.W = 0.0
        
        # ----------------------------------------
        # Apply velocity limits for TurtleBot3
        # ----------------------------------------
        
        omega_cmd = self.omega
        omega_cmd = max(min(omega_cmd, 2.84), -2.84)  # TurtleBot3 max angular velocity
        
        # ----------------------------------------
        # Publish velocity command
        # ----------------------------------------
        
        cmd = Twist()
        cmd.linear.x = self.V
        cmd.angular.z = omega_cmd
        self.cmd_vel_pub.publish(cmd)
        
        # ----------------------------------------
        # Log trajectory data
        # ----------------------------------------
        
        if self.start_time is None:
            self.start_time = time.time()
        
        current_time = time.time() - self.start_time
        if current_time - self.last_log_time >= self.log_interval:
            self.trajectory_x.append(self.x)
            self.trajectory_y.append(self.y)
            self.trajectory_psi.append(self.psi)
            self.trajectory_theta.append(self.theta)
            self.trajectory_r.append(self.r)
            self.trajectory_omega.append(self.omega)
            self.trajectory_time.append(current_time)
            self.last_log_time = current_time

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

    def stop(self):
        """Stop the robot."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def set_target(self, xt: float, yt: float):
        """Set target position."""
        self.xt = xt
        self.yt = yt
        self.get_logger().info(f'Target set to ({xt}, {yt})')

    def print_state(self):
        """Print current state for debugging."""
        self.get_logger().info(
            f'Pos: ({self.x:.2f}, {self.y:.2f}) | '
            f'psi: {math.degrees(self.psi):.1f}° | '
            f'r: {self.r:.2f} | '
            f'theta: {math.degrees(self.theta):.1f}° | '
            f'omega: {self.omega:.2f}'
        )

    # ============================================
    # PLOTTING FUNCTIONS
    # ============================================
    
    def plot_trajectory(self, save_path=None):
        """
        Plot the recorded trajectory with heading arrows.
        
        Args:
            save_path: Optional path to save the figure
        """
        if len(self.trajectory_x) < 2:
            self.get_logger().warn('Not enough data to plot trajectory')
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 12))
        
        # ---- Plot 1: Trajectory with heading arrows ----
        ax1 = axes[0, 0]
        
        # Plot orbit circles
        q = np.linspace(0, 2*np.pi, 100)
        ax1.plot(self.xt + self.rd*np.cos(q), self.yt + self.rd*np.sin(q), 
                'g--', linewidth=2, label=f'Desired orbit (rd={self.rd}m)')
        ax1.plot(self.xt + self.ra*np.cos(q), self.yt + self.ra*np.sin(q), 
                'r--', linewidth=1.5, label=f'Avoidance (ra={self.ra}m)')
        ax1.plot(self.xt + self.rs*np.cos(q), self.yt + self.rs*np.sin(q), 
                'orange', linestyle='--', linewidth=1, label=f'Standoff (rs={self.rs}m)')
        
        # Plot trajectory
        ax1.plot(self.trajectory_x, self.trajectory_y, 'b-', linewidth=1.5, label='Trajectory')
        
        # Plot heading arrows (every nth point)
        arrow_step = max(1, len(self.trajectory_x) // 20)
        arrow_length = 0.15
        for i in range(0, len(self.trajectory_x), arrow_step):
            dx = arrow_length * np.cos(self.trajectory_psi[i])
            dy = arrow_length * np.sin(self.trajectory_psi[i])
            ax1.arrow(self.trajectory_x[i], self.trajectory_y[i], dx, dy,
                     head_width=0.05, head_length=0.03, fc='red', ec='red', alpha=0.7)
        
        # Plot start and end points
        ax1.plot(self.trajectory_x[0], self.trajectory_y[0], 'go', markersize=12, label='Start')
        ax1.plot(self.trajectory_x[-1], self.trajectory_y[-1], 'rs', markersize=12, label='End')
        
        # Plot target
        ax1.plot(self.xt, self.yt, 'k*', markersize=15, label='Target')
        
        ax1.set_xlabel('X (m)', fontsize=12)
        ax1.set_ylabel('Y (m)', fontsize=12)
        ax1.set_title('Trajectory with Heading', fontsize=14)
        ax1.legend(loc='upper right', fontsize=9)
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        ax1.set_aspect('equal')
        
        # ---- Plot 2: Theta vs Time ----
        ax2 = axes[0, 1]
        theta_deg = np.array(self.trajectory_theta) * 180 / np.pi
        ax2.plot(self.trajectory_time, theta_deg, 'b-', linewidth=2)
        ax2.axhline(y=90, color='g', linestyle='--', linewidth=1.5, label='Target (90°)')
        ax2.set_xlabel('Time (s)', fontsize=12)
        ax2.set_ylabel('θ (deg)', fontsize=12)
        ax2.set_title('Theta vs Time', fontsize=14)
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # ---- Plot 3: Radius vs Time ----
        ax3 = axes[1, 0]
        ax3.plot(self.trajectory_time, self.trajectory_r, 'b-', linewidth=2)
        ax3.axhline(y=self.rd, color='g', linestyle='--', linewidth=1.5, label=f'Desired (rd={self.rd}m)')
        ax3.axhline(y=self.ra, color='r', linestyle='--', linewidth=1, label=f'Avoidance (ra={self.ra}m)')
        ax3.set_xlabel('Time (s)', fontsize=12)
        ax3.set_ylabel('r (m)', fontsize=12)
        ax3.set_title('Distance to Target vs Time', fontsize=14)
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # ---- Plot 4: Omega vs Time ----
        ax4 = axes[1, 1]
        ax4.plot(self.trajectory_time, self.trajectory_omega, 'b-', linewidth=2)
        ax4.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        ax4.set_xlabel('Time (s)', fontsize=12)
        ax4.set_ylabel('ω (rad/s)', fontsize=12)
        ax4.set_title('Angular Velocity vs Time', fontsize=14)
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            self.get_logger().info(f'Figure saved to {save_path}')
        
        plt.show()
    
    def animate_trajectory(self, interval=50, save_path=None):
        """
        Create an animation of the robot movement.
        
        Args:
            interval: Animation interval in milliseconds
            save_path: Optional path to save animation (as .gif or .mp4)
        """
        if len(self.trajectory_x) < 2:
            self.get_logger().warn('Not enough data to animate')
            return
        
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Plot orbit circles (static)
        q = np.linspace(0, 2*np.pi, 100)
        ax.plot(self.xt + self.rd*np.cos(q), self.yt + self.rd*np.sin(q), 
               'g--', linewidth=2, label=f'Desired orbit (rd={self.rd}m)')
        ax.plot(self.xt + self.ra*np.cos(q), self.yt + self.ra*np.sin(q), 
               'r--', linewidth=1.5, label=f'Avoidance (ra={self.ra}m)')
        
        # Plot target
        ax.plot(self.xt, self.yt, 'k*', markersize=20, label='Target')
        
        # Initialize trajectory line and robot marker
        trajectory_line, = ax.plot([], [], 'b-', linewidth=1.5, alpha=0.7)
        robot_marker, = ax.plot([], [], 'ro', markersize=10)
        heading_arrow = ax.arrow(0, 0, 0, 0, head_width=0.08, head_length=0.05, 
                                  fc='red', ec='red')
        
        # Time text
        time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, fontsize=12,
                           verticalalignment='top', fontfamily='monospace',
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # Set axis limits
        all_x = self.trajectory_x + [self.xt]
        all_y = self.trajectory_y + [self.yt]
        margin = 0.5
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('TurtleBot3 Circumnavigation Animation', fontsize=14)
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        def init():
            trajectory_line.set_data([], [])
            robot_marker.set_data([], [])
            time_text.set_text('')
            return trajectory_line, robot_marker, time_text
        
        def update(frame):
            nonlocal heading_arrow
            
            # Update trajectory
            trajectory_line.set_data(self.trajectory_x[:frame+1], self.trajectory_y[:frame+1])
            
            # Update robot position
            robot_marker.set_data([self.trajectory_x[frame]], [self.trajectory_y[frame]])
            
            # Update heading arrow
            heading_arrow.remove()
            arrow_length = 0.2
            dx = arrow_length * np.cos(self.trajectory_psi[frame])
            dy = arrow_length * np.sin(self.trajectory_psi[frame])
            heading_arrow = ax.arrow(self.trajectory_x[frame], self.trajectory_y[frame], 
                                     dx, dy, head_width=0.08, head_length=0.05, 
                                     fc='red', ec='red')
            
            # Update time text
            time_text.set_text(f't = {self.trajectory_time[frame]:.1f}s\n'
                              f'r = {self.trajectory_r[frame]:.2f}m\n'
                              f'θ = {np.degrees(self.trajectory_theta[frame]):.1f}°')
            
            return trajectory_line, robot_marker, time_text, heading_arrow
        
        anim = FuncAnimation(fig, update, frames=len(self.trajectory_x),
                            init_func=init, blit=False, interval=interval, repeat=True)
        
        if save_path:
            if save_path.endswith('.gif'):
                anim.save(save_path, writer='pillow', fps=1000//interval)
            else:
                anim.save(save_path, writer='ffmpeg', fps=1000//interval)
            self.get_logger().info(f'Animation saved to {save_path}')
        
        plt.show()


# ============================================
# MAIN FUNCTION
# ============================================

def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    # Create controller
    # For single robot: CircumnavigationController('')
    # For multi-robot: CircumnavigationController('tb3_1')
    controller = CircumnavigationController('tb3_1')
    
    try:
        controller.get_logger().info('Waiting for odometry data...')
        
        # Wait for first odometry message
        while not controller.odom_received:
            rclpy.spin_once(controller, timeout_sec=0.1)
        
        controller.get_logger().info('Odometry received! Controller starting.')
        controller.get_logger().info(f'Initial position: ({controller.x:.2f}, {controller.y:.2f})')
        controller.get_logger().info('Press Ctrl+C to stop.')
        
        # Main spin
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print('\nShutting down...')
        
    finally:
        # Try to stop robot (ignore errors if context is invalid)
        try:
            controller.stop()
        except:
            pass
        
        # Destroy node and shutdown (ignore errors)
        try:
            controller.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass
        
        # Plot results after ROS cleanup
        if len(controller.trajectory_x) > 2:
            print(f'Recorded {len(controller.trajectory_x)} trajectory points')
            print('Generating plots...')
            
            # Static plot with all data
            # controller.plot_trajectory()
            
            # Uncomment below for animation
            controller.animate_trajectory(interval=50)


if __name__ == '__main__':
    main()
