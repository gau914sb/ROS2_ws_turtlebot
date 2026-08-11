#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class StampedMover(Node):
    def __init__(self):
        super().__init__('stamped_mover')
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.target_distance = 0.7
        self.speed = 0.1
        self.duration = 5.0 
        self.start_time = self.get_clock().now()
        
        # Use a timer to keep publishing
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("StampedMover node has been started.")

    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        if elapsed < self.duration:
            msg.twist.linear.x = self.speed
        else:
            msg.twist.linear.x = 0.0
            self.get_logger().info("Target time reached. Stopping.")
            self.timer.cancel()
            
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StampedMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
