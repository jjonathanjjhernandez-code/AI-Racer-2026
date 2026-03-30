from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch simple publisher and subscriber nodes"""

    return LaunchDescription([
        # Launch simple publisher node
        Node(
            package='my_python_pkg',
            executable='simple_publisher',
            name='simple_publisher',
            output='screen',
            emulate_tty=True,
        ),

        # Launch simple subscriber node
        Node(
            package='my_python_pkg',
            executable='simple_subscriber',
            name='simple_subscriber',
            output='screen',
            emulate_tty=True,
        ),
    ])