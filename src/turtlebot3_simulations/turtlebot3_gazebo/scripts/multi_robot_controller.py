#!/usr/bin/env python3
"""
Multi-Robot Controller for TurtleBot3
Provides position feedback and velocity control for 4 robots
Author: Custom implementation based on TurtleBot3 examples
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class MultiRobotController(Node):
    """
    Controller for managing multiple TurtleBot3 robots
    Features:
    - Position and orientation feedback from odometry
    - Velocity command publishing
    - Simple formation control patterns
    - Individual robot status monitoring
    """

    def __init__(self):
        super().__init__('multi_robot_controller')
        
        # Robot configuration
        self.num_robots = 4
        self.namespace_prefix = 'tb3'  # Use lowercase to match actual topics from multi_robot.launch.py
        
        # Data storage for each robot
        self.robot_data = {}
        for i in range(self.num_robots):
            robot_name = f'{self.namespace_prefix}_{i+1}'
            self.robot_data[robot_name] = {
                'odom': None,
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
                'velocity': {'linear': 0.0, 'angular': 0.0},
                'joint_state': None,
                'target_velocity': {'linear': 0.0, 'angular': 0.0}
            }
        
        # QoS profile for reliable communication
        qos = QoSProfile(depth=10)
        
        # Create publishers and subscribers for each robot
        self.cmd_vel_publishers = {}
        self.odom_subscribers = {}
        self.joint_state_subscribers = {}
        
        for i in range(self.num_robots):
            robot_name = f'{self.namespace_prefix}_{i+1}'
            
            # Command velocity publishers
            self.cmd_vel_publishers[robot_name] = self.create_publisher(
                Twist, f'/{robot_name}/cmd_vel', qos)
            
            # Odometry subscribers
            self.odom_subscribers[robot_name] = self.create_subscription(
                Odometry, f'/{robot_name}/odom', 
                self._create_odom_callback(robot_name), qos)
            
            # Joint state subscribers (optional - for wheel info)
            self.joint_state_subscribers[robot_name] = self.create_subscription(
                JointState, f'/{robot_name}/joint_states',
                self._create_joint_callback(robot_name), qos)
        
        # Status publisher for monitoring
        self.status_publisher = self.create_publisher(String, '/multi_robot_status', qos)
        
        # Control timers
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        self.status_timer = self.create_timer(1.0, self.publish_status)   # 1Hz status update
        
        # Control mode and parameters
        self.control_mode = 'manual'  # 'manual', 'formation', 'circle', 'stop'
        self.formation_spacing = 2.0
        self.circle_radius = 2.0
        self.circle_speed = 0.5
        self.start_time = time.time()
        
        self.get_logger().info(f'Multi-Robot Controller initialized for {self.num_robots} robots')
        self.print_usage()

    def _create_odom_callback(self, robot_name):
        """Create a proper callback closure for odometry"""
        def callback(msg):
            self.odom_callback(msg, robot_name)
        return callback
    
    def _create_joint_callback(self, robot_name):
        """Create a proper callback closure for joint states"""
        def callback(msg):
            self.joint_state_callback(msg, robot_name)
        return callback

    def odom_callback(self, msg, robot_name):
        """Process odometry data from robots"""
        self.robot_data[robot_name]['odom'] = msg
        
        # Extract position
        self.robot_data[robot_name]['position']['x'] = msg.pose.pose.position.x
        self.robot_data[robot_name]['position']['y'] = msg.pose.pose.position.y
        self.robot_data[robot_name]['position']['z'] = msg.pose.pose.position.z
        
        # Extract orientation (convert quaternion to Euler)
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.robot_data[robot_name]['orientation']['yaw'] = yaw
        
        # Extract velocity
        self.robot_data[robot_name]['velocity']['linear'] = msg.twist.twist.linear.x
        self.robot_data[robot_name]['velocity']['angular'] = msg.twist.twist.angular.z

    def joint_state_callback(self, msg, robot_name):
        """Process joint state data from robots"""
        self.robot_data[robot_name]['joint_state'] = msg

    def set_robot_velocity(self, robot_name, linear_x=0.0, angular_z=0.0):
        """Set velocity for a specific robot"""
        if robot_name in self.cmd_vel_publishers:
            cmd_vel = Twist()
            cmd_vel.linear.x = float(linear_x)
            cmd_vel.angular.z = float(angular_z)
            self.cmd_vel_publishers[robot_name].publish(cmd_vel)
            
            # Store target velocity for monitoring
            self.robot_data[robot_name]['target_velocity']['linear'] = linear_x
            self.robot_data[robot_name]['target_velocity']['angular'] = angular_z

    def stop_all_robots(self):
        """Emergency stop for all robots"""
        for robot_name in self.robot_data.keys():
            self.set_robot_velocity(robot_name, 0.0, 0.0)
        self.get_logger().info('All robots stopped')

    def set_formation_square(self, side_length=2.0, linear_vel=0.2):
        """Move robots to maintain square formation"""
        target_positions = [
            [-side_length/2, -side_length/2],  # tb3_1: bottom-left
            [side_length/2, -side_length/2],   # tb3_2: bottom-right
            [side_length/2, side_length/2],    # tb3_3: top-right
            [-side_length/2, side_length/2]    # tb3_4: top-left
        ]
        
        for i, robot_name in enumerate(sorted(self.robot_data.keys())):
            if self.robot_data[robot_name]['odom'] is not None:
                current_x = self.robot_data[robot_name]['position']['x']
                current_y = self.robot_data[robot_name]['position']['y']
                target_x, target_y = target_positions[i]
                
                # Simple proportional control to target position
                dx = target_x - current_x
                dy = target_y - current_y
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance > 0.1:  # Move only if not at target
                    # Calculate desired heading
                    target_yaw = math.atan2(dy, dx)
                    current_yaw = self.robot_data[robot_name]['orientation']['yaw']
                    yaw_error = target_yaw - current_yaw
                    
                    # Normalize angle
                    while yaw_error > math.pi:
                        yaw_error -= 2*math.pi
                    while yaw_error < -math.pi:
                        yaw_error += 2*math.pi
                    
                    # Control gains
                    linear_gain = 0.5
                    angular_gain = 1.0
                    
                    # Calculate velocities
                    linear_cmd = min(linear_gain * distance, linear_vel)
                    angular_cmd = angular_gain * yaw_error
                    
                    self.set_robot_velocity(robot_name, linear_cmd, angular_cmd)
                else:
                    self.set_robot_velocity(robot_name, 0.0, 0.0)

    def set_circle_formation(self, radius=2.0, angular_vel=0.3):
        """Move robots in circular formation"""
        elapsed_time = time.time() - self.start_time
        
        for i, robot_name in enumerate(sorted(self.robot_data.keys())):
            # Each robot gets a different starting angle
            angle_offset = i * (2 * math.pi / self.num_robots)
            current_angle = angular_vel * elapsed_time + angle_offset
            
            # Target position on circle
            target_x = radius * math.cos(current_angle)
            target_y = radius * math.sin(current_angle)
            
            if self.robot_data[robot_name]['odom'] is not None:
                current_x = self.robot_data[robot_name]['position']['x']
                current_y = self.robot_data[robot_name]['position']['y']
                
                # Calculate movement to maintain circular motion
                dx = target_x - current_x
                dy = target_y - current_y
                
                # Proportional control
                linear_vel = 0.3 * math.sqrt(dx*dx + dy*dy)
                angular_vel_cmd = angular_vel + 0.5 * math.atan2(dy, dx)
                
                self.set_robot_velocity(robot_name, linear_vel, angular_vel_cmd)

    def control_loop(self):
        """Main control loop - 10Hz"""
        if self.control_mode == 'stop':
            self.stop_all_robots()
        elif self.control_mode == 'formation':
            self.set_formation_square()
        elif self.control_mode == 'circle':
            self.set_circle_formation()
        # 'manual' mode - controlled externally or via other methods

    def publish_status(self):
        """Publish system status - 1Hz"""
        status_msg = String()
        status_lines = [f"Multi-Robot Controller Status (Mode: {self.control_mode})"]
        
        for robot_name in sorted(self.robot_data.keys()):
            data = self.robot_data[robot_name]
            if data['odom'] is not None:
                pos = data['position']
                vel = data['velocity']
                target = data['target_velocity']
                status_lines.append(
                    f"{robot_name}: Pos[{pos['x']:.2f}, {pos['y']:.2f}] "
                    f"Vel[L:{vel['linear']:.2f}, A:{vel['angular']:.2f}] "
                    f"Target[L:{target['linear']:.2f}, A:{target['angular']:.2f}]"
                )
            else:
                status_lines.append(f"{robot_name}: No odometry data")
        
        status_msg.data = '\n'.join(status_lines)
        self.status_publisher.publish(status_msg)

    def print_usage(self):
        """Print usage instructions"""
        self.get_logger().info("""
        Multi-Robot Controller Commands:
        ===============================
        
        Python API Usage:
        - controller.set_robot_velocity('tb3_1', 0.2, 0.5)  # Set individual velocity
        - controller.stop_all_robots()                      # Stop all robots
        - controller.control_mode = 'formation'             # Enable formation control
        - controller.control_mode = 'circle'                # Enable circle formation
        - controller.control_mode = 'manual'                # Manual control mode
        - controller.control_mode = 'stop'                  # Stop all robots
        
        Available robot names: tb3_1, tb3_2, tb3_3, tb3_4
        
        Position feedback available in controller.robot_data[robot_name]['position']
        Velocity feedback available in controller.robot_data[robot_name]['velocity']
        """)

    def get_robot_position(self, robot_name):
        """Get current position of a specific robot"""
        if robot_name in self.robot_data and self.robot_data[robot_name]['odom'] is not None:
            return self.robot_data[robot_name]['position']
        return None

    def get_all_positions(self):
        """Get positions of all robots"""
        positions = {}
        for robot_name in self.robot_data.keys():
            pos = self.get_robot_position(robot_name)
            if pos is not None:
                positions[robot_name] = pos
        return positions


def main(args=None):
    rclpy.init(args=args)
    
    controller = MultiRobotController()
    
    try:
        controller.get_logger().info("Starting multi-robot controller...")
        
        # Wait a bit for initial odometry data
        controller.get_logger().info("Waiting for robot odometry data...")
        for i in range(10):  # Wait up to 10 seconds
            rclpy.spin_once(controller, timeout_sec=1.0)
            
            # Check how many robots have odometry
            odom_count = sum(1 for data in controller.robot_data.values() if data['odom'] is not None)
            controller.get_logger().info(f"Odometry received from {odom_count}/4 robots")
            
            if odom_count >= 2:  # Continue if at least 2 robots have odom
                break
        
        controller.get_logger().info("Controller ready! Use the API to control robots:")
        controller.get_logger().info("- controller.set_robot_velocity('tb3_1', 0.2, 0.0)")
        controller.get_logger().info("- controller.control_mode = 'formation'")
        controller.get_logger().info("- controller.control_mode = 'circle'")
        controller.get_logger().info("Press Ctrl+C to stop")
        
        # Main control loop
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down multi-robot controller...")
        try:
            controller.stop_all_robots()
        except:
            pass  # Ignore errors during shutdown
    
    finally:
        try:
            controller.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()