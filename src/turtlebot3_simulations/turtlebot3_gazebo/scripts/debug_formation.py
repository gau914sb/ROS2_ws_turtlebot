#!/usr/bin/env python3
"""Debug formation control"""

import rclpy
import time
from multi_robot_controller import MultiRobotController

def debug_formation():
    rclpy.init()
    controller = MultiRobotController()
    
    try:
        print("Starting controller and waiting for odometry data...")
        
        # Wait for all robots to have odometry data
        for i in range(10):
            rclpy.spin_once(controller, timeout_sec=1.0)
            ready_count = 0
            for robot_name in controller.robot_data.keys():
                if controller.robot_data[robot_name]['odom'] is not None:
                    ready_count += 1
            
            print(f"Ready robots: {ready_count}/4")
            if ready_count == 4:
                break
        
        # Show current positions
        print("\n=== Current Positions ===")
        for robot_name in sorted(controller.robot_data.keys()):
            pos = controller.robot_data[robot_name]['position']
            print(f"{robot_name}: x={pos['x']:.3f}, y={pos['y']:.3f}")
        
        # Show target positions for formation
        target_positions = [
            [-1.0, -1.0],  # tb3_1: bottom-left
            [1.0, -1.0],   # tb3_2: bottom-right
            [1.0, 1.0],    # tb3_3: top-right
            [-1.0, 1.0]    # tb3_4: top-left
        ]
        
        print("\n=== Target Positions ===")
        for i, robot_name in enumerate(sorted(controller.robot_data.keys())):
            target_x, target_y = target_positions[i]
            print(f"{robot_name}: target x={target_x:.3f}, y={target_y:.3f}")
        
        print("\n=== Distance to Targets ===")
        for i, robot_name in enumerate(sorted(controller.robot_data.keys())):
            current_x = controller.robot_data[robot_name]['position']['x']
            current_y = controller.robot_data[robot_name]['position']['y']
            target_x, target_y = target_positions[i]
            
            dx = target_x - current_x
            dy = target_y - current_y
            distance = (dx*dx + dy*dy)**0.5
            print(f"{robot_name}: distance = {distance:.3f}m, dx={dx:.3f}, dy={dy:.3f}")
        
        print("\nTesting formation control for 15 seconds...")
        print("Watch the robots move towards their target positions!")
        
        controller.control_mode = 'formation'
        
        start_time = time.time()
        while time.time() - start_time < 15.0:
            rclpy.spin_once(controller, timeout_sec=0.1)
            
            # Print status every 3 seconds
            if int((time.time() - start_time)) % 3 == 0:
                elapsed = time.time() - start_time
                print(f"\nTime: {elapsed:.1f}s")
                for i, robot_name in enumerate(sorted(controller.robot_data.keys())):
                    pos = controller.robot_data[robot_name]['position']
                    target_x, target_y = target_positions[i]
                    dx = target_x - pos['x']
                    dy = target_y - pos['y']
                    distance = (dx*dx + dy*dy)**0.5
                    print(f"  {robot_name}: pos=({pos['x']:.2f}, {pos['y']:.2f}) dist={distance:.2f}m")
                time.sleep(1)  # Avoid spam
        
        print("\nStopping formation control...")
        controller.control_mode = 'manual'
        controller.stop_all_robots()
        time.sleep(1)
        
        print("Formation control test completed!")
        
    except KeyboardInterrupt:
        print("Test interrupted")
        controller.stop_all_robots()
    
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    debug_formation()