from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='driveline_urc',
            executable='GpsDriver_node',
            name='GpsDriver',
            output='screen'
        ),
        Node(
            package='driveline_urc',
            executable='MotorCtr_node',
            name='MotorController',
            output='screen'
        )
    ])
