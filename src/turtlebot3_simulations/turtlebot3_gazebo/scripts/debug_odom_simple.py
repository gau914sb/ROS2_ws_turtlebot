#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class SimpleOdomListener(Node):
    def __init__(self):
        super().__init__('simple_odom_listener')
        self.received_count = {
            'tb3_1': 0,
            'tb3_2': 0, 
            'tb3_3': 0,
            'tb3_4': 0
        }
        
        # Create subscribers for each robot
        for i in range(1, 5):
            robot_name = f'tb3_{i}'
            self.create_subscription(
                Odometry, 
                f'/{robot_name}/odom',
                lambda msg, name=robot_name: self.odom_callback(msg, name),
                10
            )
            
        self.timer = self.create_timer(2.0, self.print_stats)
        
    def odom_callback(self, msg, robot_name):
        self.received_count[robot_name] += 1
        if self.received_count[robot_name] == 1:
            print(f"First odom received from {robot_name}: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}")
            
    def print_stats(self):
        print("Odometry message counts:")
        for robot, count in self.received_count.items():
            print(f"  {robot}: {count} messages")

def main():
    rclpy.init()
    node = SimpleOdomListener()
    
    print("Listening for odometry messages for 10 seconds...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()