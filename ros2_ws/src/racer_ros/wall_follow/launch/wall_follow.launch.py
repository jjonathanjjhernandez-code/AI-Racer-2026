#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch wall following node with F1TENTH GYM ROS simulator
    """

    # Get package directories
    wall_follow_dir = get_package_share_directory('wall_follow')

    # Path to parameter file
    params_file = os.path.join(wall_follow_dir, 'config', 'wall_follow_params.yaml')

    # Declare launch arguments
    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='1.0',
        description='Proportional gain'
    )

    ki_arg = DeclareLaunchArgument(
        'ki',
        default_value='0.0',
        description='Integral gain'
    )

    kd_arg = DeclareLaunchArgument(
        'kd',
        default_value='0.0',
        description='Derivative gain'
    )

    desired_distance_arg = DeclareLaunchArgument(
        'desired_distance',
        default_value='1.0',
        description='Desired distance from wall (meters)'
    )

    # Wall follow node
    wall_follow_node = Node(
        package='wall_follow',
        executable='wall_follow_node',
        name='wall_follow_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
        ]
    )

    return LaunchDescription([
        kp_arg,
        ki_arg,
        kd_arg,
        desired_distance_arg,
        wall_follow_node,
    ])