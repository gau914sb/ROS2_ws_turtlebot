#!/usr/bin/env python3
"""
4-robot spawning launch file for TurtleBot3 with ArUco markers for identification
Modifies the SDF file to add ArUco marker visuals dynamically
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

def add_aruco_marker_to_sdf(root, robot_idx, texture_path):
    """Add ArUco marker link and joint to SDF model using Ogre material script."""
    model = root.find('model')
    
    # Create marker link
    marker_link = ET.SubElement(model, 'link')
    marker_link.set('name', 'marker_link')
    
    # Position the link on top of the robot
    link_pose = ET.SubElement(marker_link, 'pose')
    link_pose.text = '-0.03 0 0.190 0 0 0'  # Just above the LiDAR
    
    # Add inertial properties (required for valid link)
    inertial = ET.SubElement(marker_link, 'inertial')
    mass = ET.SubElement(inertial, 'mass')
    mass.text = '0.001'
    inertia = ET.SubElement(inertial, 'inertia')
    ixx = ET.SubElement(inertia, 'ixx')
    ixx.text = '0.000001'
    iyy = ET.SubElement(inertia, 'iyy')
    iyy.text = '0.000001'
    izz = ET.SubElement(inertia, 'izz')
    izz.text = '0.000001'
    
    # Visual element with ArUco texture via Ogre material script
    visual = ET.SubElement(marker_link, 'visual')
    visual.set('name', 'aruco_visual')
    
    geometry = ET.SubElement(visual, 'geometry')
    box = ET.SubElement(geometry, 'box')
    size = ET.SubElement(box, 'size')
    size.text = '0.08 0.08 0.001'  # Thin flat marker
    
    # Material with Ogre script reference
    # Materials are installed in /usr/share/gazebo-11/media/materials/
    # Gazebo loads all .material files from this path automatically
    material = ET.SubElement(visual, 'material')
    script = ET.SubElement(material, 'script')
    
    # Use Gazebo's default media path (no explicit URI needed)
    uri = ET.SubElement(script, 'uri')
    uri.text = '__default__'
    
    # Material name (must match definition in aruco.material)
    name = ET.SubElement(script, 'name')
    name.text = f'Aruco/Marker{robot_idx}'
    
    # Create joint to attach marker_link to base_link
    marker_joint = ET.SubElement(model, 'joint')
    marker_joint.set('name', 'marker_joint')
    marker_joint.set('type', 'fixed')
    
    parent = ET.SubElement(marker_joint, 'parent')
    parent.text = 'base_link'
    
    child = ET.SubElement(marker_joint, 'child')
    child.text = 'marker_link'
    
    return root

# Keep color markers as fallback
ROBOT_COLORS = [
    (1.0, 0.0, 0.0, 1.0),  # Robot 1: Red
    (0.0, 1.0, 0.0, 1.0),  # Robot 2: Green
    (0.0, 0.0, 1.0, 1.0),  # Robot 3: Blue
    (1.0, 1.0, 0.0, 1.0),  # Robot 4: Yellow
]

def add_color_marker_to_sdf(root, robot_idx):
    """Add colored marker link and joint to SDF model (fallback)."""
    model = root.find('model')
    r, g, b, a = ROBOT_COLORS[robot_idx]
    
    marker_link = ET.SubElement(model, 'link')
    marker_link.set('name', 'marker_link')
    
    link_pose = ET.SubElement(marker_link, 'pose')
    link_pose.text = '-0.03 0 0.190 0 0 0'
    
    inertial = ET.SubElement(marker_link, 'inertial')
    mass = ET.SubElement(inertial, 'mass')
    mass.text = '0.001'
    inertia = ET.SubElement(inertial, 'inertia')
    for tag in ['ixx', 'iyy', 'izz']:
        elem = ET.SubElement(inertia, tag)
        elem.text = '0.000001'
    
    visual = ET.SubElement(marker_link, 'visual')
    visual.set('name', 'marker_visual')
    
    geometry = ET.SubElement(visual, 'geometry')
    box = ET.SubElement(geometry, 'box')
    size = ET.SubElement(box, 'size')
    size.text = '0.08 0.08 0.01'
    
    material = ET.SubElement(visual, 'material')
    ambient = ET.SubElement(material, 'ambient')
    ambient.text = f'{r} {g} {b} {a}'
    diffuse = ET.SubElement(material, 'diffuse')
    diffuse.text = f'{r} {g} {b} {a}'
    
    marker_joint = ET.SubElement(model, 'joint')
    marker_joint.set('name', 'marker_joint')
    marker_joint.set('type', 'fixed')
    parent = ET.SubElement(marker_joint, 'parent')
    parent.text = 'base_link'
    child = ET.SubElement(marker_joint, 'child')
    child.text = 'marker_link'
    
    return root

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    
    # Use ArUco markers (True) or color markers (False)
    USE_ARUCO_MARKERS = True

    # Simple configuration
    number_of_robots = 4
    namespace = 'tb3'
    
    # [x, y, yaw]
    pose = [[1, 1, 0],        # Robot 1
            [3.7, 1, 0],      # Robot 2
            [3.7, 1.7, 0],    # Robot 3
            [1, 1.7, 0]]      # Robot 4

    # Package directories
    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    sdf_path = os.path.join(turtlebot3_gazebo_dir, 'models', model_folder, 'model.sdf')
    save_path = os.path.join(turtlebot3_gazebo_dir, 'models', model_folder, 'tmp')
    launch_file_dir = os.path.join(turtlebot3_gazebo_dir, 'launch')
    
    # ArUco texture paths (using installed location)
    aruco_textures = [
        os.path.join(turtlebot3_description_dir, 'media', 'materials', 'textures', f'agent_{i}.png')
        for i in range(4)
    ]
    
    # NOTE: Do NOT modify GAZEBO_RESOURCE_PATH or GAZEBO_MODEL_PATH here!
    # Doing so causes gzclient to crash with "Assertion 'px != 0' failed"
    # Materials must be installed to /usr/share/gazebo-11/media/materials/

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'empty_world.world')

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
        robot_id = count + 1
        robot_namespace = f'{namespace}_{robot_id}'
        
        # Robot state publisher for each robot
        robot_state_publisher_cmd_list.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'frame_prefix': robot_namespace
                }.items()
            )
        )

        # Parse and modify SDF for each robot
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        
        # Modify frame names for namespace
        for odom_frame_tag in root.iter('odometry_frame'):
            odom_frame_tag.text = f'{robot_namespace}/odom'
        for base_frame_tag in root.iter('robot_base_frame'):
            base_frame_tag.text = f'{robot_namespace}/base_footprint'
        for scan_frame_tag in root.iter('frame_name'):
            scan_frame_tag.text = f'{robot_namespace}/base_scan'
        
        # Unique plugin names to avoid conflicts  
        for plugin in root.iter('plugin'):
            if plugin.get('name') == 'turtlebot3_imu':
                plugin.set('name', f'{robot_namespace}_imu')
            elif plugin.get('name') == 'turtlebot3_diff_drive':
                plugin.set('name', f'{robot_namespace}_diff_drive')
            elif plugin.get('name') == 'turtlebot3_joint_state':
                plugin.set('name', f'{robot_namespace}_joint_state')
            elif plugin.get('name') == 'turtlebot3_laserscan':
                plugin.set('name', f'{robot_namespace}_laserscan')
        
        # Add marker (ArUco texture or color)
        if USE_ARUCO_MARKERS:
            root = add_aruco_marker_to_sdf(root, count, aruco_textures[count])
        else:
            root = add_color_marker_to_sdf(root, count)
        
        # Save modified SDF
        sdf_modified = ET.tostring(root, encoding='unicode')
        sdf_modified = '<?xml version="1.0" ?>\n' + sdf_modified
        sdf_file_path = f'{save_path}{robot_id}.sdf'
        with open(sdf_file_path, 'w') as file:
            file.write(sdf_modified)

        # Spawn robot
        spawn_turtlebot_cmd_list.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'multi_spawn_with_yaw.launch.py')
                ),
                launch_arguments={
                    'x_pose': str(pose[count][0]),
                    'y_pose': str(pose[count][1]),
                    'yaw_pose': str(pose[count][2]),
                    'robot_name': f'{TURTLEBOT3_MODEL}_{robot_id}',
                    'namespace': robot_namespace,
                    'sdf_path': sdf_file_path
                }.items()
            )
        )

    ld = LaunchDescription()
    
    # Add Gazebo
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    
    # Cleanup on shutdown
    ld.add_action(RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event, context: [
                os.remove(f'{save_path}{c+1}.sdf') 
                for c in range(number_of_robots) 
                if os.path.exists(f'{save_path}{c+1}.sdf')
            ]
        )
    ))
    
    # Add robots
    for count, spawn_turtlebot_cmd in enumerate(spawn_turtlebot_cmd_list, start=1):
        ld.add_action(GroupAction([
            PushRosNamespace(f'{namespace}_{count}'),
            robot_state_publisher_cmd_list[count-1],
            spawn_turtlebot_cmd
        ]))

    return ld
