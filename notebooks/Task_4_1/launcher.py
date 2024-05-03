from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pbvs_package',
            executable='feature_extractor',
            name='feature_extractor_node'
        ),
        Node(
            package='pbvs_package',
            executable='controller',
            name='controller_node'
        ),
        Node(
            package='pbvs_package',
            executable='actuator',
            name='actuator_node'
        )
    ])
