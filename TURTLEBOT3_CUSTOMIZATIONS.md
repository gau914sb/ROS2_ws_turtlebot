# TurtleBot3 Customizations Documentation

This document maintains all customizations made to the TurtleBot3 simulation and configuration files in this workspace.

## Table of Contents
- [Launch File Structure](#launch-file-structure)
- [Sensor Modifications](#sensor-modifications)
- [Performance Optimizations](#performance-optimizations)
- [Multi-Robot Deployment](#multi-robot-deployment)
- [Files Modified](#files-modified)

---

## Launch File Structure

### Basic TurtleBot3 Launch File Components

TurtleBot3 launch files follow a standard ROS2 launch structure. Here are the core components:

#### 1. Essential Imports
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
```

#### 2. Core Function Structure
```python
def generate_launch_description():
    # Environment variables
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']  # burger, waffle, waffle_pi
    
    # Launch configurations (parameters)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    # Package directories
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Component definitions...
    
    # Launch description assembly
    ld = LaunchDescription()
    ld.add_action(component1)
    ld.add_action(component2)
    return ld
```

#### 3. Standard Components

**Gazebo Server & Client:**
```python
gzserver_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
    ),
    launch_arguments={'world': world_path}.items()
)

gzclient_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    )
)
```

**Robot State Publisher:**
```python
robot_state_publisher_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
    ),
    launch_arguments={'use_sim_time': use_sim_time}.items()
)
```

**Robot Spawning:**
```python
spawn_turtlebot_cmd = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', robot_name,
        '-file', urdf_path,
        '-x', x_pose,
        '-y', y_pose,
        '-z', '0.01'
    ],
    output='screen',
)
```

#### 4. Multi-Robot Specific Patterns

**Namespace Management:**
- Use `GroupAction` with `PushRosNamespace` for isolated robot control
- Modify SDF files dynamically for frame naming
- Handle odometry and scan frame prefixes

**Dynamic SDF Modification:**
```python
import xml.etree.ElementTree as ET

tree = ET.parse(urdf_path)
root = tree.getroot()
for odom_frame_tag in root.iter('odometry_frame'):
    odom_frame_tag.text = f'{namespace}_{robot_id}/odom'
```

---

## Sensor Modifications

### Lidar Scanner Disabled
**Date:** January 13, 2026  
**Performance Impact:** FPS improved from 29 to 62 in Gazebo simulation  
**Reason:** Significant performance improvement for applications that don't require lidar data  

#### What was changed:
The lidar sensor (LDS - Laser Distance Sensor) has been disabled in the TurtleBot3 Burger model.

#### How to disable/enable:
In `/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf`, locate the sensor configuration:

```xml
<sensor name="hls_lfcd_lds" type="ray">
  <always_on>false</always_on>        <!-- Set to 'true' to enable -->
  <visualize>false</visualize>        <!-- Set to 'true' to show rays -->
  <pose>-0.032 0 0.171 0 0 0</pose>
  <update_rate>5</update_rate>
  <!-- ... rest of sensor config ... -->
</sensor>
```

**To enable lidar:**
- Change `<always_on>false</always_on>` to `<always_on>true</always_on>`
- Change `<visualize>false</visualize>` to `<visualize>true</visualize>` (optional, for ray visualization)

**To disable lidar:**
- Set `<always_on>false</always_on>`
- Set `<visualize>false</visualize>`

#### Benefits of this approach:
- ✅ Clean and reversible (no commenting/uncommenting code blocks)
- ✅ Maintains model structure integrity
- ✅ Standard Gazebo sensor management practice
- ✅ Significant performance improvement (2x FPS increase observed)

---

## Performance Optimizations

### Gazebo Simulation Performance
- **Lidar disabled:** +113% FPS increase (29 → 62 FPS)
- **Impact:** Real-time factor improved significantly
- **Use case:** Suitable for navigation testing without lidar dependency

---

## Files Modified

### Core Model Files
1. **`/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf`**
   - Modified sensor parameters: `always_on` and `visualize`
   - Line ~141-142: Changed lidar sensor activation flags

### URDF Files (No changes needed)
- **`/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf`**
   - No modifications required (URDF link structure preserved)
   - Sensor behavior controlled via SDF model file

---

## Usage Notes

### When to use this configuration:
- Development and testing scenarios where lidar is not required
- Performance-critical applications
- Learning ROS2 navigation with alternative sensors
- Gazebo simulations on lower-end hardware

### When to re-enable lidar:
- SLAM (Simultaneous Localization and Mapping) applications
- Autonomous navigation requiring obstacle detection
- Sensor fusion projects
- Real-world deployment preparation

### Verification Commands:
```bash
# Check if lidar topic is active
ros2 topic list | grep scan

# Monitor Gazebo performance
# Check FPS in Gazebo GUI status bar

# Launch simulation
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

---

## Multi-Robot Deployment ✅

### Working Solutions:
1. **Original multi_robot.launch.py**: Works perfectly for 4 robots
   ```bash
   ros2 launch turtlebot3_gazebo multi_robot.launch.py
   ```

2. **Custom spawn_4_bots.launch.py**: Simplified version with yaw control
   ```bash
   ros2 launch turtlebot3_gazebo spawn_4_bots.launch.py
   ```

### Features of Custom Launch File:
- **Fixed positions**: 4 robots in square formation ([-1,-1], [1,-1], [1,1], [-1,1])
- **Individual orientations**: Each robot faces different direction (east, north, west, south)
- **Namespace isolation**: tb3_1, tb3_2, tb3_3, tb3_4
- **Lidar disabled**: All performance optimizations maintained
- **Direct spawn control**: Uses spawn_entity.py with `-Y` parameter for yaw

### Yaw Orientation Configuration:
```python
# [x, y, yaw] - yaw in radians
pose = [[-1, -1, 0],      # Robot 1: facing east (→)
        [1, -1, 1.57],    # Robot 2: facing north (↑) 
        [1, 1, 3.14],     # Robot 3: facing west (←)
        [-1, 1, -1.57]]   # Robot 4: facing south (↓)
```

### spawn_entity.py Location:
- **Main executable**: `/opt/ros/humble/lib/gazebo_ros/spawn_entity.py`
- **Package**: `gazebo_ros` (part of ROS2 Humble)
- **Usage**: Direct spawning with position and orientation control
- **Key parameters**: `-x`, `-y`, `-z` (position), `-Y` (yaw angle in radians)

---

## Multi-Robot Controller ✅

### Controller Files:
1. **Main Controller**: [`multi_robot_controller.py`](src/turtlebot3_simulations/turtlebot3_gazebo/scripts/multi_robot_controller.py)
2. **Example Usage**: [`multi_robot_example.py`](src/turtlebot3_simulations/turtlebot3_gazebo/scripts/multi_robot_example.py)

### Features:
- **Position Feedback**: Real-time odometry data from all 4 robots
- **Velocity Control**: Individual robot command velocity publishing  
- **Formation Control**: Built-in square and circle formation patterns
- **Status Monitoring**: System-wide status reporting at 1Hz
- **Emergency Stop**: Stop all robots with single command

### Usage Examples:

#### Basic Controller Setup:
```python
from multi_robot_controller import MultiRobotController
import rclpy

rclpy.init()
controller = MultiRobotController()

# Individual robot control
controller.set_robot_velocity('tb3_1', 0.2, 0.0)  # linear, angular
controller.set_robot_velocity('tb3_2', 0.0, 0.5)  # turn in place

# Get position feedback
positions = controller.get_all_positions()
print(f"tb3_1 position: {positions['tb3_1']}")

# Formation control
controller.control_mode = 'formation'  # square formation
controller.control_mode = 'circle'     # circle formation
controller.control_mode = 'manual'     # manual control

# Emergency stop
controller.stop_all_robots()
```

#### Running the Controller:
```bash
# Terminal 1: Start robots
ros2 launch turtlebot3_gazebo spawn_4_bots.launch.py

# Terminal 2: Run controller
cd /home/gaurav/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/scripts/
python3 multi_robot_controller.py

# Terminal 3: Run examples (optional)
python3 multi_robot_example.py

# Monitor status
ros2 topic echo /multi_robot_status
```

### Available Topics Per Robot:
- **Command**: `/{robot_name}/cmd_vel` (geometry_msgs/Twist)
- **Odometry**: `/{robot_name}/odom` (nav_msgs/Odometry)  
- **Joint States**: `/{robot_name}/joint_states` (sensor_msgs/JointState)
- **Status**: `/multi_robot_status` (std_msgs/String)

### Controller Modes:
- `'manual'`: Direct velocity control via API
- `'formation'`: Automatic square formation maintenance
- `'circle'`: Circular formation with rotation
- `'stop'`: Emergency stop all robots

### Position Feedback Structure:
```python
robot_data[robot_name] = {
    'position': {'x': float, 'y': float, 'z': float},
    'orientation': {'yaw': float},  # radians
    'velocity': {'linear': float, 'angular': float},
    'target_velocity': {'linear': float, 'angular': float}
}
```

---

## Maintenance Log

| Date & Time | Modification | Author | Notes |
|-------------|-------------|---------|-------|
| 2026-01-13 14:30 UTC | Disabled lidar sensor | - | Performance optimization: 29→62 FPS |
| 2026-01-13 14:45 UTC | Created documentation | - | Initial documentation setup |
| 2026-01-13 14:47 UTC | Added timestamp tracking | - | Enhanced maintenance log with timestamps |
| 2026-01-13 15:15 UTC | Added launch file structure guide | - | Comprehensive launch file analysis and multi-robot spawn guide |
| 2026-01-13 16:31 UTC | Successfully tested multi-robot deployment | - | Original multi_robot.launch.py works with lidar disabled (4 robots spawned) |
| 2026-01-13 16:45 UTC | Created custom spawn_4_bots.launch.py | - | Simplified multi-robot launch with yaw control using spawn_entity.py |
| 2026-01-13 17:00 UTC | Created multi-robot controller system | - | Position feedback and velocity control for all 4 robots with formation patterns |
| 2026-01-18 12:00 UTC | Created unicycle_controller.py | Copilot | Clean template for single TurtleBot unicycle control with global frame position/velocity feedback |
| 2026-01-18 12:15 UTC | Created multi_unicycle_controller.py | Copilot | Template for 4 independent TurtleBots with parallel unicycle control |
| 2026-01-18 12:30 UTC | Created circumnavigation_controller.py | Copilot | Implemented MATLAB circumnavigation control law (circum_experimental_multiple_entry.m) |
| 2026-01-18 13:00 UTC | Added trajectory plotting to circumnavigation_controller.py | Copilot | 4-panel static plot (trajectory, θ, r, ω vs time) and animation capability |
| 2026-01-18 13:15 UTC | Fixed shutdown error handling | Copilot | Graceful ROS context cleanup to avoid errors on Ctrl+C |
| 2026-01-22 14:00 UTC | Enhanced multi_unicycle_controller.py plotting | Copilot | Added neighbor connection visualization (dotted for initial, solid for current/final positions) |
| 2026-01-22 14:30 UTC | Added auto-save plots with timestamps | Copilot | Plots auto-save to ~/ros2_ws/plots/{timestamp}/ on shutdown (trajectory.png, distances.png, combined_animation.gif) |
| 2026-01-22 15:00 UTC | Created combined animation feature | Copilot | Side-by-side animation (1x2 layout) showing trajectory + distance plots synchronized |

---

**Last Updated:** January 22, 2026 15:00 UTC  
**ROS2 Version:** Humble  
**Gazebo Version:** 11+  
**TurtleBot3 Model:** Burger

## How to Update This Documentation

When making changes to TurtleBot3 configurations:

1. **Document the change** in the appropriate section above
2. **Add an entry** to the Maintenance Log with:
   - Current date and time (UTC format: YYYY-MM-DD HH:MM UTC)
   - Brief description of modification
   - Author/contributor name
   - Impact notes or reasoning
3. **Update the "Last Updated" timestamp** at the bottom
4. **Commit changes** to version control with descriptive message

### Template for new entries:
```markdown
| YYYY-MM-DD HH:MM UTC | Brief description | Your Name | Impact/Notes |
```