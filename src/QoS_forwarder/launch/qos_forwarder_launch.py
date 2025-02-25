from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='QoS_forwarder',  # Replace with your package name
            executable='qos_forwarder_node',  # Replace with your node executable name
            name='qos_forwarder_node',
            output='screen',
            parameters=[{
                'use_sim_time': False  # Add any parameters you want to set
            }]
        )
    ])