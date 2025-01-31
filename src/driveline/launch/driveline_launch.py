import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('driveline_urc'),
        'urdf',
        'driveline_urc.urdf'
    )

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}],
        ),
    ])
