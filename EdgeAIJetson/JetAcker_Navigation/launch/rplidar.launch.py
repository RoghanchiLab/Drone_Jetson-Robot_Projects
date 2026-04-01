#!/usr/bin/env python3
"""
RPLidar S2 launch file for JetAcker Autonomous Navigation.

Launches the sllidar_ros2 driver node configured for RPLidar S2 (1 Mbaud).
Publishes sensor_msgs/LaserScan on /scan consumed by SLAM Toolbox and Nav2 costmaps.

NOTE: The sllidar_ros2 package provides two executables:
  - 'sllidar_node'        : standalone node (used here)
  - 'sllidar_composition' : composable node for zero-copy component containers
  Switch to sllidar_composition + ComposableNodeContainer if CPU-constrained.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the RPLidar S2 (typically /dev/rplidar or /dev/ttyUSB0).',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )

    rplidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type':     'serial',
            'serial_port':      LaunchConfiguration('serial_port'),
            'serial_baudrate':  1000000,    # RPLidar S2 requires 1 Mbaud
            'frame_id':         'laser',    # must match URDF lidar link
            'inverted':         False,
            'angle_compensate': True,
            # DenseBoost: higher point density vs Standard; good for indoor SLAM.
            # Switch to 'Standard' if CPU usage is too high.
            'scan_mode':        'DenseBoost',
            'use_sim_time':     LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        use_sim_time_arg,
        rplidar_node,
    ])
