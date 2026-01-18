#!/usr/bin/env python3
"""Quick test for formation control"""

import rclpy
import time
from multi_robot_controller import MultiRobotController

def test_formation():
    rclpy.init()
    controller = MultiRobotController()
    
    try:
        print("Starting controller and waiting for odometry data...")
        
        # Wait for odometry data
        for i in range(20):
            rclpy.spin_once(controller, timeout_sec=0.5)
            
            # Check if all robots have odometry data
            all_ready = True
            for robot_name in controller.robot_data.keys():
                if controller.robot_data[robot_name]['odom'] is None:
                    all_ready = False
                    break
            
            if all_ready:
                print(f"All robots ready after {i+1} attempts")
                break
            else:
                print(f"Attempt {i+1}: Waiting for odometry data...")
        
        # Show current positions
        positions = controller.get_all_positions()
        print("\nCurrent robot positions:")
        for name, pos in positions.items():
            if pos:
                print(f"  {name}: x={pos['x']:.2f}, y={pos['y']:.2f}")
        
        print("\nTesting formation control for 10 seconds...")
        controller.control_mode = 'formation'
        
        # Run formation control
        start_time = time.time()
        while time.time() - start_time < 10.0:
            rclpy.spin_once(controller, timeout_sec=0.1)
        
        print("\nStopping all robots...")
        controller.control_mode = 'manual'
        controller.stop_all_robots()
        
        print("Test completed!")
        
    except KeyboardInterrupt:
        print("Test interrupted")
        controller.stop_all_robots()
    
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    test_formation()