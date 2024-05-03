from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pbvs_package',
            executable='pbvs_feature_node',
            name='feature_extractor_node',
            parameters=[
                {'featureParams': [[1, 1, -2], [2, 0.5], [-1, -1, 2]]}
            ]
        ),
        Node(
            package='pbvs_package',
            executable='pbvs_controller_node',
            name='controller_node',
            parameters=[
                {'controllerParams': [0, 0, 1]}
            ]
        ),
        Node(
            package='pbvs_package',
            executable='pbvs_actuator_node',
            name='actuator_node',
            parameters=[
                {'actuatorParams': [1, 1, -2]}
            ]
        )
    ])
