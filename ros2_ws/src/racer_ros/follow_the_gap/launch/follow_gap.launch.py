#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch Follow the Gap node
    """

    # Get package directory
    pkg_dir = get_package_share_directory('follow_the_gap')
    params_file = os.path.join(pkg_dir, 'config', 'gap_params.yaml')

    # Declare launch arguments
    bubble_radius_arg = DeclareLaunchArgument(
        'bubble_radius',
        default_value='0.3',
        description='Safety bubble radius (meters)'
    )

    speed_max_arg = DeclareLaunchArgument(
        'speed_max',
        default_value='2.0',
        description='Maximum speed (m/s)'
    )

    steering_gain_arg = DeclareLaunchArgument(
        'steering_gain',
        default_value='1.0',
        description='Steering proportional gain'
    )

    # Reactive node
    reactive_node = Node(
        package='follow_the_gap',
        executable='reactive_node',
        name='reactive_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            {
                'bubble_radius': LaunchConfiguration('bubble_radius'),
                'speed_max': LaunchConfiguration('speed_max'),
                'steering_gain': LaunchConfiguration('steering_gain'),
            }
        ]
    )

    return LaunchDescription([
        bubble_radius_arg,
        speed_max_arg,
        steering_gain_arg,
        reactive_node,
    ])