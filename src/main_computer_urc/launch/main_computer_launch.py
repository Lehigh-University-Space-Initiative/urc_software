from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ## Setup for State publishing

    urdf_file = PathJoinSubstitution(
        [FindPackageShare("main_computer_urc"), "description", "robot.urdf.xacro"]
    )
    rviz_file = PathJoinSubstitution(
        [FindPackageShare("main_computer_urc"), "description", "robot.rviz"]
    )
    # rviz_file = PathJoinSubstitution(
    #     [FindPackageShare("moveit_config_urc"), "config", "moveit.rviz"]
    # )
    # ros2_control_config = PathJoinSubstitution(
    #     [FindPackageShare("main_computer_urc"), "description", "config", "ros2_control.yaml"]
    # )

    robot_description_content = Command([
        'xacro ',
        urdf_file
    ])
    robot_description = {"robot_description": robot_description_content}    


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file],
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )


    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        joint_state_publisher_node,
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
        # Node(
        #     package='main_computer_urc',
        #     executable='VideoStreamer_node',
        #     name='VideoStreamer',
        #     output='screen'
        # ),
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
