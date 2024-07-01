from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    # Create the feature extractor node
    feature_extractor_node = Node(
        package='pbvs',
        executable='feature_extractor',
        name='feature_extractor'
    )
    
    # Create the controller node
    controller_node = Node(
        package='pbvs',
        executable='controller',
        name='controller'
    )
    
    # Create the actuator node
    actuator_node = Node(
        package='pbvs',
        executable='actuator',
        name='actuator'
    )
    
    # Add the nodes to the launch description
    ld.add_action(actuator_node)
    ld.add_action(controller_node)
    ld.add_action(feature_extractor_node)
    
    return ld