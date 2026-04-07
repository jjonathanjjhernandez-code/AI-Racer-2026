#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    movement_dir = get_package_share_directory('movement_node')
    aeb_dir = get_package_share_directory('AEB_System')
    
    safety_params = os.path.join(aeb_dir, 'config', 'safety_params.yaml')
    reactive_params = os.path.join(movement_dir, 'config', 'movement.yaml')
    
    safety_node = Node(
        package = 'AEB_System',
        executable= 'safety_node',
        name='safety_node',
        output='screen',
        emulate_tty=True,
        parameters=[safety_params],
    )
    
    movement_node = Node(
        package = 'movement_node',
        executable= 'trajectory',
        name='trajectory',
        output='screen',
        emulate_tty=True,
        parameters=[reactive_params],
    )
    
    return LaunchDescription([
        safety_node,
        movement_node
    ])