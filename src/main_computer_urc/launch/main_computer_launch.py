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
        Node(
            package='image_transport',
            executable='republish',
            name='republish',
            output='screen',
            arguments=[
                'compressed',  # Input transport type
                '--ros-args',
                '--remap', 'in:=/video_stream',
                '--remap', 'out/compressed:=/video_stream/compressed',
            ],
        ),
    ])
