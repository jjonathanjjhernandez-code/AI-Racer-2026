#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    follow_the_gap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('follow_the_gap'),
                'launch',
                'follow_gap.launch.py'
            )
        )
    )

    aeb_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('AEB_System'),
                'launch',
                'safety_node.launch.py'
            )
        )
    )

    return LaunchDescription([
        aeb_system,
        follow_the_gap,
    ])