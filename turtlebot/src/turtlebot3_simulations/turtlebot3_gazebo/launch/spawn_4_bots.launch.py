#!/usr/bin/env python3
"""
spawn_4_bots.launch.py
======================
Launches Gazebo Harmonic with 4 TurtleBot3 Burger robots.

KEY APPROACH: generates per-robot SDF files with ABSOLUTE gz topic names.
e.g. /bot_0/cmd_vel, /bot_0/odom — avoiding the gz model-scoping ambiguity
that causes gz topics to appear unscoped (/odom, /cmd_vel) in some versions.

Robots:
  bot_0  at (-1.0, -1.0)  yaw=0
  bot_1  at ( 1.0, -1.0)  yaw=1.57
  bot_2  at ( 1.0,  1.0)  yaw=3.14
  bot_3  at (-1.0,  1.0)  yaw=-1.57

ROS2 Jazzy / Gazebo Harmonic
"""

import os
import re
import tempfile
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


# ── Robot configurations ──────────────────────────────────────────────────────
# Real-hardware initial conditions, Case-4 (collision avoidance ON).
# x, y, yaw -- first logged row of APS Report/Figures/Case-4/log_data.csv.
# ROBOTS = [
#     ('bot_0', -0.3034, -0.0093, 0.6830),
#     ('bot_1', -0.6136, 0.4315, -0.9821),
#     ('bot_2', 0.1132, -0.2382, 2.1886),
#     ('bot_3', -0.3801, -0.3463, -2.1991),
# ]

# ROBOTS = [
#     ('bot_0', -0.3034, -0.0093, math.pi/4),
#     ('bot_1', -0.6136, 0.4315, -math.pi/4),
#     ('bot_2', 0.1132, -0.2382, 2*math.pi/3),
#     ('bot_3', -0.3801, -0.3463, -2*math.pi/3),
# ]

# Real-hardware initial conditions, Case-1b (collision avoidance OFF).
# x, y, yaw -- first logged row of APS Report/Figures/Case-1b/log_data.csv.
# Swap this in for ROBOTS above to test Case-1b instead.
# ROBOTS = [
#     ('bot_0', 0.6705, 0.1398, 1.4427),
#     ('bot_1', -0.6370, 0.1786, 1.6645),
#     ('bot_2', -0.6128, -0.1347, -1.2970),
#     ('bot_3', 0.6413, -0.1755, -1.4689),
# ]

# Case-3/Case-4 shared IC family, rounded -- exactly matches
# experiment/move_to_initial_ekf.py's active TARGETS dict (the "no
# constraints case" block). Case-3 and Case-4 share this same real-hardware
# IC family; this rounded version sits closer to Case-4's exact IC above
# (within ~1-5 cm / ~6-11 deg) than to Case-3's (up to ~34 deg off on
# bot_0's heading). Swap this in for ROBOTS above to match move_to_initial_ekf.py.
ROBOTS = [
    ('bot_0', -0.3, 0.0, math.pi/4),
    ('bot_1', -0.6, 0.4, -math.pi/4),
    ('bot_2', 0.1, -0.2, 2*math.pi/3),
    ('bot_3', -0.4, -0.4, -2*math.pi/3),
]


# ROBOTS = [
#     ('bot_0', -0.3, 0.0, math.pi/4),
#     ('bot_1', -0.6, 0.4, -math.pi/4),
#     ('bot_2', 0.1, -0.2, 2*math.pi/3),
#     ('bot_3', -0.4, -0.4, -2*math.pi/3),
# ]

# ROBOTS = [
#     ('bot_0', 1.32, 0.44, 0.0),
#     ('bot_1', 0.88, 0.88, math.pi/2),
#     ('bot_2', 0.44, 0.44, math.pi),
#     ('bot_3', 0.88, 0.0, 3*math.pi/2),
# ]

# def make_robot_sdf(base_sdf_path: str, ns: str) -> str:
#     """
#     Read the base burger SDF and rewrite all plugin topic names to be
#     absolute (e.g. /bot_0/cmd_vel) so that Gazebo Harmonic publishes to
#     the correct, per-robot gz topics regardless of model-scoping behaviour.
#     Returns the path to the generated temp SDF file.
#     """
#     with open(base_sdf_path, 'r') as f:
#         sdf = f.read()

#     # DiffDrive plugin topics (relative → absolute per-robot)
#     sdf = re.sub(
#         r'<topic>cmd_vel</topic>',
#         f'<topic>/{ns}/cmd_vel</topic>',
#         sdf
#     )
#     sdf = re.sub(
#         r'<odom_topic>odom</odom_topic>',
#         f'<odom_topic>/{ns}/odom</odom_topic>',
#         sdf
#     )
#     sdf = re.sub(
#         r'<tf_topic>/tf</tf_topic>',
#         f'<tf_topic>/{ns}/tf</tf_topic>',
#         sdf
#     )

#     # JointStatePublisher topic
#     sdf = re.sub(
#         r'<topic>joint_states</topic>',
#         f'<topic>/{ns}/joint_states</topic>',
#         sdf
#     )

#     # IMU sensor topic
#     sdf = re.sub(
#         r'<topic>imu</topic>',
#         f'<topic>/{ns}/imu</topic>',
#         sdf
#     )

#     # LiDAR scan topic
#     sdf = re.sub(
#         r'<topic>scan</topic>',
#         f'<topic>/{ns}/scan</topic>',
#         sdf
#     )

#     # Write to a named temp file (persist for the lifetime of the process)
#     tmp = tempfile.NamedTemporaryFile(
#         mode='w', suffix=f'_{ns}.sdf', delete=False, prefix='/tmp/tb3_'
#     )
#     tmp.write(sdf)
#     tmp.flush()
#     tmp.close()
#     return tmp.name

def make_robot_sdf(base_sdf_path: str, ns: str) -> str:
    """
    Read the base burger SDF and rewrite all plugin topic names to be
    absolute (e.g. /bot_0/cmd_vel). Also injects Ground Truth Odometry.
    """
    with open(base_sdf_path, 'r') as f:
        sdf = f.read()

    # DiffDrive plugin topics
    sdf = re.sub(
        r'<topic>cmd_vel</topic>',
        f'<topic>/{ns}/cmd_vel</topic>',
        sdf
    )
    sdf = re.sub(
        r'<odom_topic>odom</odom_topic>',
        f'<odom_topic>/{ns}/odom</odom_topic>',
        sdf
    )
    sdf = re.sub(
        r'<tf_topic>/tf</tf_topic>',
        f'<tf_topic>/{ns}/tf</tf_topic>',
        sdf
    )

    # SILENCE THE DRIFTING DIFF-DRIVE ODOMETRY
    # Replace whatever frequency is there with 0
    sdf = re.sub(
        r'<odom_publish_frequency>.*?</odom_publish_frequency>',
        '<odom_publish_frequency>0</odom_publish_frequency>',
        sdf
    )

    # JointStatePublisher, IMU, and LiDAR topics
    sdf = re.sub(r'<topic>joint_states</topic>', f'<topic>/{ns}/joint_states</topic>', sdf)
    sdf = re.sub(r'<topic>imu</topic>', f'<topic>/{ns}/imu</topic>', sdf)
    sdf = re.sub(r'<topic>scan</topic>', f'<topic>/{ns}/scan</topic>', sdf)

    # ==========================================
    # INJECT GROUND TRUTH ODOMETRY PLUGIN
    # ==========================================
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
    # Insert the new plugin right before the model closes
    sdf = sdf.replace('</model>', ground_truth_plugin + '\n</model>')

    # Write to a named temp file
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix=f'_{ns}.sdf', delete=False, prefix='/tmp/tb3_'
    )
    tmp.write(sdf)
    tmp.flush()
    tmp.close()
    return tmp.name

def generate_launch_description():
    pkg = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = os.path.join(pkg, 'worlds', 'empty_world.world')

    base_sdf = os.path.join(
        pkg, 'models', 'turtlebot3_burger', 'model.sdf'
    )
    urdf_path = os.path.join(pkg, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg, 'models')
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v2 {world}',
            'on_exit_shutdown': 'true',
        }.items()
    )

    robot_actions = []

    for idx, (ns, x, y, yaw) in enumerate(ROBOTS):
        delay = 5.0 + idx * 2.0

        # Per-robot SDF with absolute topic names
        robot_sdf = make_robot_sdf(base_sdf, ns)

        spawner = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{ns}',
            arguments=[
                '-name', ns,
                '-file', robot_sdf,
                '-x', str(x),
                '-y', str(y),
                '-z', '0.01',
                '-Y', str(yaw),
            ],
            output='screen',
        )

        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'frame_prefix': f'{ns}/',
            }],
        )

        bridge_cfg = os.path.join(pkg, 'params', f'{ns}_bridge.yaml')
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{ns}',
            arguments=[
                '--ros-args',
                '-p', f'config_file:={bridge_cfg}',
            ],
            output='screen',
        )

        robot_group = TimerAction(
            period=delay,
            actions=[
                GroupAction(actions=[
                    PushRosNamespace(ns),
                    rsp,
                ]),
                spawner,
                bridge,
            ]
        )
        robot_actions.append(robot_group)

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'))
    ld.add_action(set_gz_resource_path)
    ld.add_action(gz_sim)
    for action in robot_actions:
        ld.add_action(action)

    return ld
