from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    message_arg = DeclareLaunchArgument(
        'message',
        default_value='Hello from the LUSI rover software team!',
        description='Message the node will print each tick'
    )
    rate_arg = DeclareLaunchArgument(
        'rate_hz',
        default_value='1.0',
        description='Rate (in Hz) at which to print the message'
    )
    hello = Node(
        package='lusi_training',
        executable='hello_node',
        name='hello_node',
        parameters=[{
            'message': LaunchConfiguration('message'),
            'rate_hz': LaunchConfiguration('rate_hz')
        }],
        output='screen',
    )

    return LaunchDescription([message_arg, rate_arg, hello])


