from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("main_computer_urc"), "description", "urdf", "robot.urdf.xacro"]
    )
    ros2_control_config = PathJoinSubstitution(
        [FindPackageShare("main_computer_urc"), "description", "config", "ros2_control.yaml"]
    )

    robot_description_content = Command([
        'xacro ',
        urdf_file
    ])
    robot_description = {"robot_description": robot_description_content}

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, ros2_control_config],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
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
