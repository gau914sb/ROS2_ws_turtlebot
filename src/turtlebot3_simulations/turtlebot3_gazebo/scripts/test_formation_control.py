#!/usr/bin/env python3

import rclpy
from multi_robot_controller import MultiRobotController
import time

def test_formation_control():
    rclpy.init()
    
    controller = MultiRobotController()
    
    try:
        print("=== Multi-Robot Formation Control Test ===")
        print("Waiting for robot odometry...")
        
        # Wait for odometry data
        for i in range(10):
            rclpy.spin_once(controller, timeout_sec=1.0)
            odom_count = sum(1 for data in controller.robot_data.values() if data['odom'] is not None)
            print(f"Odometry from {odom_count}/4 robots")
            if odom_count >= 2:
                break
        
        print("\n=== Current Robot Positions ===")
        for name in sorted(controller.robot_data.keys()):
            data = controller.robot_data[name]
            if data['odom'] is not None:
                pos = data['position']
                print(f"{name}: x={pos['x']:.2f}, y={pos['y']:.2f}")
            else:
                print(f"{name}: No odometry data")
        
        print("\n=== Testing Individual Control ===")
        
        # Test individual robot movement
        print("Moving tb3_1 forward...")
        controller.set_robot_velocity('tb3_1', 0.2, 0.0)
        for _ in range(10):
            rclpy.spin_once(controller, timeout_sec=0.1)
        
        print("Stopping tb3_1, turning tb3_2...")
        controller.set_robot_velocity('tb3_1', 0.0, 0.0)
        controller.set_robot_velocity('tb3_2', 0.0, 0.5)
        for _ in range(10):
            rclpy.spin_once(controller, timeout_sec=0.1)
        
        controller.stop_all_robots()
        print("Individual control test complete!")
        
        print("\n=== Testing Formation Control ===")
        print("Activating square formation...")
        controller.control_mode = 'formation'
        
        # Let formation control run for 10 seconds
        start_time = time.time()
        while time.time() - start_time < 10.0:
            rclpy.spin_once(controller, timeout_sec=0.1)
            
            # Print status every 2 seconds
            if int((time.time() - start_time) * 2) % 4 == 0:
                print(f"Formation control active for {time.time() - start_time:.1f}s...")
        
        print("Formation control test complete!")
        controller.control_mode = 'manual'
        controller.stop_all_robots()
        
        print("\n=== Final Positions ===")
        for name in sorted(controller.robot_data.keys()):
            data = controller.robot_data[name]
            if data['odom'] is not None:
                pos = data['position']
                print(f"{name}: x={pos['x']:.2f}, y={pos['y']:.2f}")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        controller.stop_all_robots()
    
    finally:
        try:
            controller.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    test_formation_control()