from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='main_computer_urc',
            executable='DriveTrainManager_node',
            name='DriveTrainManager',
            output='screen'
        ),
    ])
