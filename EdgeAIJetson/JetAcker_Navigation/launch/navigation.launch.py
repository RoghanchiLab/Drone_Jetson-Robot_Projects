#!/usr/bin/env python3
"""
JetAcker Autonomous Navigation Launch File

Integrates AckermannNavNode (cmd_vel → hardware) with the full autonomous
navigation stack: RPLidar S2, Orbbec RGB-D, SLAM Toolbox, Nvblox, Nav2.

Key difference from JetAcker_SLAM_Nav2/slam_navigation.launch.py:
  - motor_node / servo_node are NOT launched here.
  - AckermannNavNode (ackermann_nav_node.py) directly owns the hardware
    (/dev/ttyACM0 motor controller and /dev/rrc servo board) and subscribes
    to /cmd_vel to execute Nav2 motion commands via AckermannController.
  - robot_state_publisher is launched directly using the jetacker_bringup URDF
    so TF (map → odom → base_footprint) is available without motor nodes.

Architecture:
  RPLidar S2  → SLAM Toolbox → 2D map + localization
  Orbbec RGB-D → Nvblox      → 3D costmap layer (local + global)
  Nav2 (RegulatedPurePursuit + Ackermann BT: no spin, BackUp on stuck)
       │  /cmd_vel
       ▼
  AckermannNavNode → AckermannController → motors + steering servo

Usage:
  ros2 launch jetacker_navigation navigation.launch.py
  ros2 launch jetacker_navigation navigation.launch.py use_rviz:=true
  ros2 launch jetacker_navigation navigation.launch.py localization_mode:=true
  ros2 launch jetacker_navigation navigation.launch.py \\
      motor_port:=/dev/ttyACM0 steer_device:=/dev/rrc
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg     = get_package_share_directory('jetacker_bringup')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    # Directory containing this launch file → used to resolve sibling files
    this_pkg_dir = os.path.dirname(os.path.abspath(__file__))
    nav_pkg_dir  = os.path.dirname(this_pkg_dir)   # JetAcker_Navigation/

    # Custom Ackermann BT: no spin recovery, BackUp on stuck
    bt_xml_file = os.path.join(
        bringup_pkg, 'behavior_trees', 'navigate_w_replanning_and_backup.xml'
    )

    # Config files live alongside this launch directory
    config_dir = os.path.join(nav_pkg_dir, 'config')

    # ── Launch arguments ───────────────────────────────────────────────────────
    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Launch RViz2 for visualization'),
        DeclareLaunchArgument(
            'localization_mode', default_value='false',
            description='Localization only — requires an existing saved map'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/rplidar',
            description='RPLidar S2 serial port'),
        DeclareLaunchArgument(
            'motor_port', default_value='/dev/ttyACM0',
            description='STM32 motor controller serial port'),
        DeclareLaunchArgument(
            'steer_device', default_value='/dev/rrc',
            description='Steering servo board device'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(config_dir, 'nav2_params.yaml'),
            description='Nav2 parameters YAML file'),
        # AckermannNavNode tuning — override at launch to calibrate
        DeclareLaunchArgument(
            'left_rps_scale', default_value='1.0',
            description='Left motor RPS per (m/s) of cmd_vel linear.x'),
        DeclareLaunchArgument(
            'right_rps_scale', default_value='-2.0',
            description='Right motor RPS per (m/s) — negative = inverted mount'),
        DeclareLaunchArgument(
            'wheelbase', default_value='0.14',
            description='Front-to-rear axle distance [m] for Ackermann steering'),
        DeclareLaunchArgument(
            'cmd_vel_timeout', default_value='0.5',
            description='Seconds before auto-stop when cmd_vel goes silent'),
    ]

    use_sim_time      = LaunchConfiguration('use_sim_time')
    use_rviz          = LaunchConfiguration('use_rviz')
    localization_mode = LaunchConfiguration('localization_mode')
    lidar_port        = LaunchConfiguration('lidar_port')
    motor_port        = LaunchConfiguration('motor_port')
    steer_device      = LaunchConfiguration('steer_device')
    nav2_params_file  = LaunchConfiguration('nav2_params_file')

    # ── Robot State Publisher (TF tree — no motor/servo nodes) ────────────────
    # Provides map → odom → base_footprint transforms needed by Nav2.
    # Motor/servo hardware is handled exclusively by AckermannNavNode below.
    urdf_file = os.path.join(bringup_pkg, 'urdf', 'jetacker.urdf.xacro')
    robot_description = Command([
        FindExecutable(name='xacro'), ' ', urdf_file
    ])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── AckermannNavNode — cmd_vel → hardware ─────────────────────────────────
    # This node owns /dev/ttyACM0 (motors) and /dev/rrc (servo) exclusively.
    # Do NOT run motor_node / servo_node alongside this node.
    ackermann_nav_node = Node(
        executable=os.path.join(nav_pkg_dir, 'ackermann_nav_node.py'),
        name='ackermann_nav_node',
        output='screen',
        parameters=[{
            'motor_port':       motor_port,
            'steer_device':     steer_device,
            'left_rps_scale':   LaunchConfiguration('left_rps_scale'),
            'right_rps_scale':  LaunchConfiguration('right_rps_scale'),
            'wheelbase':        LaunchConfiguration('wheelbase'),
            'cmd_vel_timeout':  LaunchConfiguration('cmd_vel_timeout'),
            'use_sim_time':     use_sim_time,
        }],
    )

    # ── RPLidar S2 ────────────────────────────────────────────────────────────
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(this_pkg_dir, 'rplidar.launch.py')
        ),
        launch_arguments={
            'serial_port':  lidar_port,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ── Orbbec RGB-D camera ───────────────────────────────────────────────────
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'orbbec_camera.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    slam_toolbox_params = os.path.join(config_dir, 'slam_toolbox_params.yaml')
    nvblox_params       = os.path.join(config_dir, 'nvblox_params.yaml')

    # ── SLAM Toolbox (mapping mode) ───────────────────────────────────────────
    slam_node = Node(
        condition=UnlessCondition(localization_mode),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_toolbox_params, {'use_sim_time': use_sim_time}],
    )

    # ── SLAM Toolbox (localization mode — requires saved map) ─────────────────
    slam_loc_node = Node(
        condition=IfCondition(localization_mode),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_toolbox_params,
            {'use_sim_time': use_sim_time, 'mode': 'localization'},
        ],
    )

    # ── Nvblox — 3D depth-camera obstacle costmap ─────────────────────────────
    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=[nvblox_params],
        remappings=[
            ('depth/image',       '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
            ('color/image',       '/camera/color/image_raw'),
            ('color/camera_info', '/camera/color/camera_info'),
        ],
    )

    # ── Nav2 navigation stack ─────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params_file,
            'autostart':    'true',
            'log_level':    'info',
            # Ackermann-safe BT: no spin recovery, BackUp on stuck
            'default_nav_to_pose_bt_xml':       bt_xml_file,
            'default_nav_through_poses_bt_xml': bt_xml_file,
        }.items(),
    )

    # ── RViz2 (optional) ──────────────────────────────────────────────────────
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', os.path.join(bringup_pkg, 'rviz', 'slam_nav.rviz')
        ],
    )

    return LaunchDescription([
        LogInfo(msg='=== JetAcker Autonomous Navigation Launch ==='),
        LogInfo(msg='  Hardware driver: AckermannNavNode (ackermann_nav_node.py)'),
        LogInfo(msg='  Sensors: RPLidar S2 + Orbbec RGB-D'),
        LogInfo(msg='  Mapping: SLAM Toolbox + Nvblox'),
        LogInfo(msg='  Planning: Nav2 RegulatedPurePursuit (Ackermann BT)'),
        *declare_args,
        rsp_node,
        ackermann_nav_node,
        rplidar_launch,
        orbbec_launch,
        slam_node,
        slam_loc_node,
        nvblox_node,
        nav2_launch,
        rviz_node,
    ])
