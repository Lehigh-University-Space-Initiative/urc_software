from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('rover_description_urc')
    urdf_path = os.path.join(pkg_path, 'urdf', 'LUCY26.urdf')

    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[{'-entity', 'LUCY26', '-file', urdf_path}],
            output='screen'
        )
    ])