from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

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


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": " -r -v 3 empty.sdf"}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "rrbot_system_position",
            "-allow_renaming",
            "true",
        ],
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )


    return LaunchDescription([
        robot_state_publisher_node,
        # rviz_node,
        #joint_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        node_robot_state_publisher,
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

# <ros2_control name="${name}" type="system">
#             <hardware>
#                 <!-- By default, set up controllers for simulation. This won't work on real hardware -->
#                 <plugin>mock_components/GenericSystem</plugin>
#             </hardware>
#             <joint name="shoulder">
#                 <command_interface name="position"/>
#                 <state_interface name="position">
#                   <param name="initial_value">${initial_positions['shoulder']}</param>
#                 </state_interface>
#                 <state_interface name="velocity"/>
#             </joint>
#             <joint name="elbow">
#                 <command_interface name="position"/>
#                 <state_interface name="position">
#                   <param name="initial_value">${initial_positions['elbow']}</param>
#                 </state_interface>
#                 <state_interface name="velocity"/>
#             </joint>

#         </ros2_control>