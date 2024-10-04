from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node to handle joystick input from /dev/input/js1
        Node(
            package='joy',
            executable='joy_node',
            name='joy0_node',
            parameters=[{'dev': '/dev/input/js1', 'coalesce_interval': 0.3}],
            remappings=[('/joy', '/joy0')]
        ),
        # Node to handle joystick input from /dev/input/js2
        Node(
            package='joy',
            executable='joy_node',
            name='joy1_node',
            parameters=[{'dev': '/dev/input/js2', 'coalesce_interval': 0.3}],
            remappings=[('/joy', '/joy1')]
        ),
    ])
