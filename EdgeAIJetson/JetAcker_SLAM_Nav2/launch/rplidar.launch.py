#!/usr/bin/env python3
"""
RPLidar S2 launch file for JetAcker robot.

Launches the rplidar_ros driver node configured for RPLidar S2 (1M baud).
Publishes sensor_msgs/LaserScan on /scan for Nav2 costmaps and RTAB-Map.

NOTE: The rplidar_ros package provides two executables:
  - 'rplidar_node'        : standalone node (used here)
  - 'rplidar_composition' : composable node for component containers
  If using component containers for zero-copy transport, switch to
  rplidar_composition with a ComposableNodeContainer instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---------- Launch arguments ----------
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the RPLidar S2 (typically /dev/ttyUSB0).',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )

    # ---------- RPLidar S2 node ----------
    rplidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type':      'serial',
            'serial_port':       LaunchConfiguration('serial_port'),
            'serial_baudrate':   1000000,   # RPLidar S2 requires 1 Mbaud
            'frame_id':          'laser',   # must match URDF lidar link
            'inverted':          False,
            'angle_compensate':  True,
            # 'Sensitivity' mode gives best range/accuracy for indoor SLAM.
            # Upstream default for S2 is 'DenseBoost' (higher point density,
            # shorter range). Switch to 'Standard' if CPU-constrained.
            'scan_mode':         'DenseBoost',
            'use_sim_time':      LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        use_sim_time_arg,
        rplidar_node,
    ])
