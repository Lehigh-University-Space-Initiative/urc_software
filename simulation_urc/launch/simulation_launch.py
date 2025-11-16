from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simulation_urc',
            executable='Simulation_node',
            name='simulation_node',
            output='screen'
        )
    ])