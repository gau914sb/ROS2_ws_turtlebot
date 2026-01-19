#!/usr/bin/env python3
"""
Simple 4-robot spawning launch file for TurtleBot3
Based on the working multi_robot.launch.py pattern
"""

import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    # Simple configuration
    number_of_robots = 4
    namespace = 'tb3'
    # [x, y, yaw] - yaw in radians (0=east, π/2=north, π=west, 3π/2=south)
    # pose = [[0, 0, 0],      # Robot 1: bottom-left, facing east (right)
    #         [1.9, 0, 0],    # Robot 2: bottom-right, facing north (up) 
    #         [1.9, 1.9, 0],     # Robot 3: top-right, facing west (left)
    #         [0, 1.9, 0]]   # Robot 4: top-left, facing south (down)
    
    pose = [[1, 1, 0],      # Robot 1: bottom-left, facing east (right)
            [3.7, 1, 0],    # Robot 2: bottom-right, facing north (up) 
            [3.7, 1.7, 0],     # Robot 3: top-right, facing west (left)
            [1, 1.7, 0]]   # Robot 4: top-left, facing south (down)

    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )
    save_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'tmp'
    )
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd_list = []
    spawn_turtlebot_cmd_list = []

    for count in range(number_of_robots):
        # Robot state publisher for each robot
        robot_state_publisher_cmd_list.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'frame_prefix': f'{namespace}_{count+1}'
                }.items()
            )
        )

        # Create modified SDF for each robot (same as working original)
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        for odom_frame_tag in root.iter('odometry_frame'):
            odom_frame_tag.text = f'{namespace}_{count+1}/odom'
        for base_frame_tag in root.iter('robot_base_frame'):
            base_frame_tag.text = f'{namespace}_{count+1}/base_footprint'
        for scan_frame_tag in root.iter('frame_name'):
            scan_frame_tag.text = f'{namespace}_{count+1}/base_scan'
        
        # Unique plugin names to avoid conflicts  
        for plugin in root.iter('plugin'):
            if plugin.get('name') == 'turtlebot3_imu':
                plugin.set('name', f'{namespace}_{count+1}_imu')
            elif plugin.get('name') == 'turtlebot3_diff_drive':
                plugin.set('name', f'{namespace}_{count+1}_diff_drive')
            elif plugin.get('name') == 'turtlebot3_joint_state':
                plugin.set('name', f'{namespace}_{count+1}_joint_state')
            elif plugin.get('name') == 'turtlebot3_laserscan':
                plugin.set('name', f'{namespace}_{count+1}_laserscan')
        
        urdf_modified = ET.tostring(tree.getroot(), encoding='unicode')
        urdf_modified = '<?xml version="1.0" ?>\n'+urdf_modified
        with open(f'{save_path}{count+1}.sdf', 'w') as file:
            file.write(urdf_modified)

        # Use multi_spawn_with_yaw.launch.py for proper namespace and yaw support
        spawn_turtlebot_cmd_list.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'multi_spawn_with_yaw.launch.py')
                ),
                launch_arguments={
                    'x_pose': str(pose[count][0]),
                    'y_pose': str(pose[count][1]),
                    'yaw_pose': str(pose[count][2]),
                    'robot_name': f'{TURTLEBOT3_MODEL}_{count+1}',
                    'namespace': f'{namespace}_{count+1}',
                    'sdf_path': f'{save_path}{count+1}.sdf'
                }.items()
            )
        )

    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event,
            context: [os.remove(f'{save_path}{count+1}.sdf') for count in range(number_of_robots)]
        )
    ))
    
    for count, spawn_turtlebot_cmd in enumerate(spawn_turtlebot_cmd_list, start=1):
        ld.add_action(GroupAction([PushRosNamespace(f'{namespace}_{count}'),
                                  robot_state_publisher_cmd_list[count-1],
                                  spawn_turtlebot_cmd]))

    return ld
