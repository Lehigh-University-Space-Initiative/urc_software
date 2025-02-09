import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('shared_code_urc'),
        'urdf',
        'driveline_urc.urdf'
    )
    
    config_file = os.path.join(
        get_package_share_directory('shared_code_urc'),
        'config',
        'robot_control.yaml'
    )

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[{'robot_description': urdf_file}, config_file],
        ),
        Node(
            package='main_computer_urc',
            executable='StatusLED_node',
            name='StatusLED',
            output='screen'
        ),
        Node(
            package='main_computer_urc',
            executable='VideoStreamer_node',
            name='VideoStreamer',
            output='screen'
        ),
    ])
