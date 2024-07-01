from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (RegisterEventHandler, EmitEvent, LogInfo)
from launch.event_handlers import (OnProcessExit, OnShutdown)
from launch.events import Shutdown

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
    
    # Shutdown launcher when nodes exit
    fe_exit_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=feature_extractor_node,
            on_exit=[
                LogInfo(msg="feature_extractor exited"),
                EmitEvent(event=Shutdown(reason='feature_extractor exited'))]
        )
    )
    
    cntrl_exit_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_node,
            on_exit=[
                LogInfo(msg="controller exited"),
                EmitEvent(event=Shutdown(reason='controller exited'))]
        )
    )
    
    act_exit_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=actuator_node,
            on_exit=[
                LogInfo(msg="actuator exited"),
                EmitEvent(event=Shutdown(reason='actuator exited'))]
        )
    )
    
    ld.add_action(fe_exit_event)
    ld.add_action(cntrl_exit_event)
    ld.add_action(act_exit_event)
    
    return ld