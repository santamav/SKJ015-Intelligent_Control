from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='comm_package_test',
            executable='publisher',
            name='comm_package_test',
            output='screen'
        )
    ])