"""
JetAcker SLAM + Navigation Master Launch File
Integrates: robot drivers, RPLidar S2, Orbbec camera, SLAM Toolbox, Nvblox, Nav2

Architecture:
  RPLidar S2  → SLAM Toolbox → 2D map + localization
  Orbbec RGB-D → Nvblox      → 3D costmap layer (local + global)
  Nav2: RegulatedPurePursuit + Ackermann BT (no spin, BackUp on stuck)

Usage:
  ros2 launch jetacker_bringup slam_navigation.launch.py
  ros2 launch jetacker_bringup slam_navigation.launch.py use_rviz:=true
  ros2 launch jetacker_bringup slam_navigation.launch.py localization_mode:=true
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory('jetacker_bringup')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    # Custom Ackermann BT: no spin recovery, BackUp on stuck
    bt_xml_file = os.path.join(
        bringup_pkg, 'behavior_trees', 'navigate_w_replanning_and_backup.xml'
    )

    # ── Launch arguments ───────────────────────────────────────────────────────
    use_sim_time      = LaunchConfiguration('use_sim_time')
    use_rviz          = LaunchConfiguration('use_rviz')
    localization_mode = LaunchConfiguration('localization_mode')
    serial_port       = LaunchConfiguration('serial_port')
    lidar_port        = LaunchConfiguration('lidar_port')
    nav2_params_file  = LaunchConfiguration('nav2_params_file')

    declare_args = [
        DeclareLaunchArgument('use_sim_time',      default_value='false'),
        DeclareLaunchArgument('use_rviz',          default_value='false',
                              description='Launch RViz2 for visualization'),
        DeclareLaunchArgument('localization_mode', default_value='false',
                              description='Localization only (requires existing map)'),
        DeclareLaunchArgument('serial_port',       default_value='/dev/motors',
                              description='STM32 motor controller serial port'),
        DeclareLaunchArgument('lidar_port',        default_value='/dev/rplidar',
                              description='RPLidar serial port'),
        DeclareLaunchArgument('nav2_params_file',
                              default_value=os.path.join(bringup_pkg, 'config', 'nav2_params.yaml'),
                              description='Nav2 parameters YAML file'),
    ]

    # ── Robot bringup (RSP + motor_node + servo_node) ─────────────────────────
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'serial_port':  serial_port,
            'use_lidar':    'false',   # RPLidar launched separately below
        }.items(),
    )

    # ── RPLidar S2 ────────────────────────────────────────────────────────────
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'rplidar.launch.py')
        ),
        launch_arguments={
            'serial_port':  lidar_port,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ── Orbbec depth camera ───────────────────────────────────────────────────
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'orbbec_camera.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    slam_toolbox_params = os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml')
    nvblox_params      = os.path.join(bringup_pkg, 'config', 'nvblox_params.yaml')

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

    # ── Nvblox — 3D depth-camera obstacle costmap for Nav2 ───────────────────
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
            'log_level':    'info',   # override orbbec launch's leaked log_level:=none
            # Custom BT: Ackermann-safe (no spin) + BackUp stuck recovery
            'default_nav_to_pose_bt_xml':      bt_xml_file,
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
        LogInfo(msg='=== JetAcker SLAM Navigation Launch ==='),
        *declare_args,
        robot_launch,
        rplidar_launch,
        orbbec_launch,
        slam_node,
        slam_loc_node,
        nvblox_node,
        nav2_launch,
        rviz_node,
    ])
