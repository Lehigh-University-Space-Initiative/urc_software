from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('rover_description_urc')
    urdf_path = os.path.join(pkg_path, 'urdf', 'LUCY26.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2'
        )
    ])