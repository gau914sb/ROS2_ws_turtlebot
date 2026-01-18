#!/usr/bin/env python3
"""
Simple example script for controlling multiple TurtleBot3 robots
Shows basic usage of the MultiRobotController class
"""

import rclpy
import time
import math
from multi_robot_controller import MultiRobotController


def example_individual_control(controller):
    """Example: Control each robot individually"""
    controller.get_logger().info("Example 1: Individual robot control")
    
    # Move each robot in different directions
    controller.set_robot_velocity('tb3_1', 0.2, 0.0)    # Forward
    controller.set_robot_velocity('tb3_2', 0.0, 0.5)    # Turn left
    controller.set_robot_velocity('tb3_3', -0.2, 0.0)   # Backward  
    controller.set_robot_velocity('tb3_4', 0.0, -0.5)   # Turn right
    
    # Let them move for 3 seconds
    time.sleep(3.0)
    
    # Stop all robots
    controller.stop_all_robots()
    time.sleep(1.0)


def example_formation_control(controller):
    """Example: Formation control"""
    controller.get_logger().info("Example 2: Square formation control")
    
    # Enable formation control for 10 seconds
    controller.control_mode = 'formation'
    time.sleep(10.0)
    
    # Stop formation control
    controller.control_mode = 'manual'
    controller.stop_all_robots()


def example_circle_formation(controller):
    """Example: Circle formation"""
    controller.get_logger().info("Example 3: Circle formation control")
    
    # Enable circle formation for 15 seconds
    controller.control_mode = 'circle'
    time.sleep(15.0)
    
    # Stop circle formation
    controller.control_mode = 'manual'
    controller.stop_all_robots()


def example_position_feedback(controller):
    """Example: Read robot positions"""
    controller.get_logger().info("Example 4: Position feedback")
    
    # Get positions of all robots
    positions = controller.get_all_positions()
    
    for robot_name, pos in positions.items():
        controller.get_logger().info(
            f"{robot_name} position: x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f}"
        )


def main():
    rclpy.init()
    
    controller = MultiRobotController()
    
    try:
        # Wait for robots to be ready
        controller.get_logger().info("Waiting for robots to be ready...")
        time.sleep(2.0)
        
        # Spin once to get initial data
        rclpy.spin_once(controller, timeout_sec=1.0)
        
        # Run examples
        example_position_feedback(controller)
        
        example_individual_control(controller)
        
        example_formation_control(controller)
        
        example_circle_formation(controller)
        
        controller.get_logger().info("All examples completed!")
        
    except KeyboardInterrupt:
        controller.get_logger().info("Examples interrupted by user")
    
    finally:
        controller.stop_all_robots()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()