#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class SimpleMultiRobotController(Node):
    def __init__(self):
        super().__init__('simple_multi_robot_controller')
        
        # Simple publishers for each robot
        self.pub_tb3_1 = self.create_publisher(Twist, '/tb3_1/cmd_vel', 10)
        self.pub_tb3_2 = self.create_publisher(Twist, '/tb3_2/cmd_vel', 10)
        self.pub_tb3_3 = self.create_publisher(Twist, '/tb3_3/cmd_vel', 10)
        self.pub_tb3_4 = self.create_publisher(Twist, '/tb3_4/cmd_vel', 10)
        
        # Simple position tracking
        self.positions = {
            'tb3_1': {'x': 0, 'y': 0},
            'tb3_2': {'x': 0, 'y': 0}, 
            'tb3_3': {'x': 0, 'y': 0},
            'tb3_4': {'x': 0, 'y': 0}
        }
        
        # Odometry subscribers
        self.create_subscription(Odometry, '/tb3_1/odom', lambda msg: self.odom_callback(msg, 'tb3_1'), 10)
        self.create_subscription(Odometry, '/tb3_2/odom', lambda msg: self.odom_callback(msg, 'tb3_2'), 10)
        self.create_subscription(Odometry, '/tb3_3/odom', lambda msg: self.odom_callback(msg, 'tb3_3'), 10)
        self.create_subscription(Odometry, '/tb3_4/odom', lambda msg: self.odom_callback(msg, 'tb3_4'), 10)
        
        self.get_logger().info('Simple Multi-Robot Controller Ready!')
    
    def odom_callback(self, msg, robot_name):
        """Update robot position"""
        self.positions[robot_name]['x'] = msg.pose.pose.position.x
        self.positions[robot_name]['y'] = msg.pose.pose.position.y
    
    def move_robot(self, robot_name, linear_x, angular_z):
        """Move individual robot"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        if robot_name == 'tb3_1':
            self.pub_tb3_1.publish(twist)
        elif robot_name == 'tb3_2':
            self.pub_tb3_2.publish(twist)
        elif robot_name == 'tb3_3':
            self.pub_tb3_3.publish(twist)
        elif robot_name == 'tb3_4':
            self.pub_tb3_4.publish(twist)
    
    def stop_all(self):
        """Stop all robots"""
        for robot in ['tb3_1', 'tb3_2', 'tb3_3', 'tb3_4']:
            self.move_robot(robot, 0.0, 0.0)
    
    def square_formation(self):
        """Simple square formation"""
        self.get_logger().info('Moving to square formation...')
        
        # Move each robot to formation position with simple movements
        # tb3_1: move left and back  
        self.move_robot('tb3_1', -0.2, 0.0)
        time.sleep(2)
        self.move_robot('tb3_1', 0.0, -0.5)
        time.sleep(2)
        self.move_robot('tb3_1', 0.0, 0.0)
        
        # tb3_2: move right and back
        self.move_robot('tb3_2', 0.0, 0.5)
        time.sleep(2) 
        self.move_robot('tb3_2', -0.2, 0.0)
        time.sleep(2)
        self.move_robot('tb3_2', 0.0, 0.0)
        
        # tb3_3: move right and forward
        self.move_robot('tb3_3', 0.2, 0.0)
        time.sleep(2)
        self.move_robot('tb3_3', 0.0, 0.5)
        time.sleep(2)
        self.move_robot('tb3_3', 0.0, 0.0)
        
        # tb3_4: move left and forward
        self.move_robot('tb3_4', 0.0, -0.5)
        time.sleep(2)
        self.move_robot('tb3_4', 0.2, 0.0) 
        time.sleep(2)
        self.move_robot('tb3_4', 0.0, 0.0)
        
        self.get_logger().info('Square formation complete!')

def main():
    rclpy.init()
    controller = SimpleMultiRobotController()
    
    try:
        print("\n=== Simple Multi-Robot Controller ===")
        print("Commands:")
        print("1 - Test individual robots")
        print("2 - Square formation")
        print("0 - Stop all")
        print("q - Quit")
        
        while True:
            # Process ROS messages
            rclpy.spin_once(controller, timeout_sec=0.1)
            
            # Simple user input (non-blocking would be better but this is simple)
            try:
                # For this demo, let's just run formation automatically
                print("\nTesting individual robots...")
                controller.move_robot('tb3_1', 0.2, 0.0)
                time.sleep(1)
                controller.move_robot('tb3_1', 0.0, 0.0)
                
                controller.move_robot('tb3_2', 0.0, 0.5)
                time.sleep(1) 
                controller.move_robot('tb3_2', 0.0, 0.0)
                
                print("Individual test done!")
                time.sleep(2)
                
                print("Starting square formation...")
                controller.square_formation()
                
                print("Demo complete! Press Ctrl+C to exit")
                break
                
            except KeyboardInterrupt:
                break
    
    except KeyboardInterrupt:
        pass
    
    finally:
        controller.stop_all()
        controller.destroy_node()
        rclpy.shutdown()
        print("Controller stopped!")

if __name__ == '__main__':
    main()