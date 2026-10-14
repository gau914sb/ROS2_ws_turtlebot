#!/usr/bin/env python3
"""
spawn_case.launch.py
=====================
New launch file for the Gazebo replication of one real TurtleBot3 formation
case (Case-1b / Case-2 / Case-3 / Case-4). Does not modify the existing
turtlebot3_gazebo/launch/spawn_4_bots.launch.py -- it is modeled on it (same
per-robot absolute-topic SDF rewriting + ground-truth odometry injection
technique) but reads spawn poses and the collision-avoidance flag from a
per-case YAML (UBUNTU_GAZEBO_TASK.md section 1/4a) instead of a hardcoded
ROBOTS list, and additionally spawns two documentation cameras (top-down +
isometric) and the new case controller (controller_case.py).

Launch arguments:
  case_yaml      path to a cases/<Case>.yaml file (required)
  log_dir        directory the controller writes log_data.csv /
                 control_status.json into (required)
  camera_top_output   full path for the top-down camera MP4 (required)
  camera_iso_output   full path for the isometric camera MP4 (required)

ROS2 Jazzy / Gazebo Harmonic (gz-sim 8).
"""
import math
import os
import re
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
GAZEBO_SIM_DIR = os.path.dirname(THIS_DIR)

ROBOT_ORDER = ['bot_0', 'bot_1', 'bot_2', 'bot_3']


def make_robot_sdf(base_sdf_path: str, ns: str) -> str:
    """Same technique as turtlebot3_gazebo/launch/spawn_4_bots.launch.py's
    make_robot_sdf: rewrite plugin topics to absolute per-robot gz topics
    and inject ground-truth odometry. Copied (not imported) so this file has
    no dependency on that one."""
    with open(base_sdf_path, 'r') as f:
        sdf = f.read()

    sdf = re.sub(r'<topic>cmd_vel</topic>', f'<topic>/{ns}/cmd_vel</topic>', sdf)
    sdf = re.sub(r'<odom_topic>odom</odom_topic>', f'<odom_topic>/{ns}/odom</odom_topic>', sdf)
    sdf = re.sub(r'<tf_topic>/tf</tf_topic>', f'<tf_topic>/{ns}/tf</tf_topic>', sdf)
    sdf = re.sub(
        r'<odom_publish_frequency>.*?</odom_publish_frequency>',
        '<odom_publish_frequency>0</odom_publish_frequency>',
        sdf,
    )
    sdf = re.sub(r'<topic>joint_states</topic>', f'<topic>/{ns}/joint_states</topic>', sdf)
    sdf = re.sub(r'<topic>imu</topic>', f'<topic>/{ns}/imu</topic>', sdf)
    sdf = re.sub(r'<topic>scan</topic>', f'<topic>/{ns}/scan</topic>', sdf)

    ground_truth_plugin = f"""
    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
        <odom_frame>world</odom_frame>
        <robot_base_frame>base_footprint</robot_base_frame>
        <odom_publish_frequency>50</odom_publish_frequency>
        <odom_topic>/{ns}/ground_truth_odom</odom_topic>
        <dimensions>3</dimensions>
    </plugin>
    """
    sdf = sdf.replace('</model>', ground_truth_plugin + '\n</model>')

    tmp = tempfile.NamedTemporaryFile(mode='w', suffix=f'_{ns}.sdf', delete=False, prefix='/tmp/tb3case_')
    tmp.write(sdf)
    tmp.flush()
    tmp.close()
    return tmp.name


def make_camera_sdf(name: str, topic: str, hfov: float, width: int, height: int, update_rate: float) -> str:
    """Static camera-only model: one link with a gz-sim camera sensor,
    published on an absolute topic. No plugin needed -- the world already
    loads gz-sim-sensors-system (see empty_world.world)."""
    sdf = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <sensor name="{name}_sensor" type="camera">
        <always_on>1</always_on>
        <visualize>false</visualize>
        <update_rate>{update_rate}</update_rate>
        <topic>{topic}</topic>
        <camera name="{name}">
          <horizontal_fov>{hfov}</horizontal_fov>
          <image>
            <width>{width}</width>
            <height>{height}</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.05</near>
            <far>100</far>
          </clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
"""
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix=f'_{name}.sdf', delete=False, prefix='/tmp/tb3case_')
    tmp.write(sdf)
    tmp.flush()
    tmp.close()
    return tmp.name


def launch_setup(context, *args, **kwargs):
    case_yaml_path = LaunchConfiguration('case_yaml').perform(context)
    log_dir = LaunchConfiguration('log_dir').perform(context)
    camera_top_output = LaunchConfiguration('camera_top_output').perform(context)
    camera_iso_output = LaunchConfiguration('camera_iso_output').perform(context)

    with open(case_yaml_path, 'r') as f:
        case_cfg = yaml.safe_load(f)

    case_name = case_cfg['case']
    collision_avoidance = bool(case_cfg['collision_avoidance'])
    conv_tol = float(case_cfg.get('conv_tol', 0.080))
    max_duration = float(case_cfg.get('max_duration', 240.0))
    robots_cfg = case_cfg['robots']

    pkg = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = os.path.join(pkg, 'worlds', 'empty_world.world')
    base_sdf = os.path.join(pkg, 'models', 'turtlebot3_burger', 'model.sdf')
    urdf_path = os.path.join(pkg, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    set_gz_resource_path = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models'))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s -v2 {world}', 'on_exit_shutdown': 'true'}.items(),
    )

    actions = [set_gz_resource_path, gz_sim]

    # ---- robots, per-case IC ----
    for idx, ns in enumerate(ROBOT_ORDER):
        rc = robots_cfg[ns]
        x, y, yaw = float(rc['x']), float(rc['y']), float(rc['yaw'])
        delay = 5.0 + idx * 2.0

        robot_sdf = make_robot_sdf(base_sdf, ns)

        spawner = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{ns}',
            arguments=['-name', ns, '-file', robot_sdf, '-x', str(x), '-y', str(y), '-z', '0.01', '-Y', str(yaw)],
            output='screen',
        )
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc, 'frame_prefix': f'{ns}/'}],
        )
        bridge_cfg = os.path.join(pkg, 'params', f'{ns}_bridge.yaml')
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{ns}',
            arguments=['--ros-args', '-p', f'config_file:={bridge_cfg}'],
            output='screen',
        )
        actions.append(TimerAction(
            period=delay,
            actions=[GroupAction(actions=[PushRosNamespace(ns), rsp]), spawner, bridge],
        ))

    # ---- cameras ----
    # Top-down: straight down from height, pitch = +90 deg.
    top_sdf = make_camera_sdf('cam_top', '/cam_top/image', hfov=1.2, width=960, height=540, update_rate=15.0)
    top_pose = dict(x=0.0, y=0.0, z=6.0, roll=0.0, pitch=math.pi / 2, yaw=0.0)

    # Isometric: offset above/beside the arena, looking at its center.
    iso_x, iso_y, iso_z = 2.5, -2.5, 2.2
    target_x, target_y, target_z = 0.0, 0.0, 0.05
    iso_yaw = math.atan2(target_y - iso_y, target_x - iso_x)
    horiz = math.hypot(target_x - iso_x, target_y - iso_y)
    iso_pitch = math.atan2(iso_z - target_z, horiz)
    iso_sdf = make_camera_sdf('cam_iso', '/cam_iso/image', hfov=1.0, width=960, height=540, update_rate=15.0)
    iso_pose = dict(x=iso_x, y=iso_y, z=iso_z, roll=0.0, pitch=iso_pitch, yaw=iso_yaw)

    for name, sdf_path, pose in (('cam_top', top_sdf, top_pose), ('cam_iso', iso_sdf, iso_pose)):
        actions.append(TimerAction(
            period=5.0,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_{name}',
                arguments=[
                    '-name', name, '-file', sdf_path,
                    '-x', str(pose['x']), '-y', str(pose['y']), '-z', str(pose['z']),
                    '-R', str(pose['roll']), '-P', str(pose['pitch']), '-Y', str(pose['yaw']),
                ],
                output='screen',
            )],
        ))

    camera_bridge_cfg = os.path.join(GAZEBO_SIM_DIR, 'camera_bridge.yaml')
    actions.append(TimerAction(
        period=6.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_cameras',
            arguments=['--ros-args', '-p', f'config_file:={camera_bridge_cfg}'],
            output='screen',
        )],
    ))

    # ---- camera recorder (new file, not a ros2-installed executable -> ExecuteProcess) ----
    recorder_script = os.path.join(GAZEBO_SIM_DIR, 'camera_recorder.py')
    actions.append(TimerAction(
        period=8.0,
        actions=[ExecuteProcess(
            cmd=[
                'python3', recorder_script, '--ros-args',
                '-p', 'top_topic:=/cam_top/image',
                '-p', 'iso_topic:=/cam_iso/image',
                '-p', f'top_output:={camera_top_output}',
                '-p', f'iso_output:={camera_iso_output}',
                '-p', 'fps:=15.0',
            ],
            output='screen',
        )],
    ))

    # ---- formation controller (new file -> ExecuteProcess) ----
    controller_script = os.path.join(GAZEBO_SIM_DIR, 'controller_case.py')
    controller_start_delay = 5.0 + len(ROBOT_ORDER) * 2.0 + 5.0
    actions.append(TimerAction(
        period=controller_start_delay,
        actions=[ExecuteProcess(
            cmd=[
                'python3', controller_script, '--ros-args',
                '-p', f'case_name:={case_name}',
                '-p', f'collision_avoidance:={str(collision_avoidance).lower()}',
                '-p', f'log_dir:={log_dir}',
                '-p', f'conv_tol:={conv_tol}',
                '-p', f'max_duration:={max_duration}',
            ],
            output='screen',
        )],
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('case_yaml', description='Path to a cases/<Case>.yaml file'),
        DeclareLaunchArgument('log_dir', description='Output directory for log_data.csv / control_status.json'),
        DeclareLaunchArgument('camera_top_output', description='Full path for camera_top.mp4'),
        DeclareLaunchArgument('camera_iso_output', description='Full path for camera_iso.mp4'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
