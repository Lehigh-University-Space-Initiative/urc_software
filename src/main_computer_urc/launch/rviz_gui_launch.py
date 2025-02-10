from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("moveit_config_urc"), "config", "moveit.rviz"]
        # [FindPackageShare("main_computer_urc"), "description", "robot.rviz"]
    )
   
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file],
    )
  

    aaaaaaaah

    return LaunchDescription([
        rviz_node,
        # joint_state_publisher_gui_node,
    ])
