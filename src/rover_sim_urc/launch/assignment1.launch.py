"""Start the suspended wheel/steering assembly in Gazebo Classic on a development PC.

All actuator commands and joint states are under /assignment1.
Existing rover launches and hardware drive topics are independent of this experiment.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Load the model, then activate controllers only after a successful Gazebo spawn."""
    share = get_package_share_directory("rover_sim_urc")
    gazebo_share = get_package_share_directory("gazebo_ros")
    description = xacro.process_file(
        os.path.join(share, "description", "assignment1.urdf.xacro")
    ).toxml()
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={
            "world": os.path.join(share, "worlds", "assignment1.world"),
            "gui": LaunchConfiguration("gui"),
            "verbose": "true",
        }.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        namespace="assignment1", name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": description, "use_sim_time": True}],
        remappings=[("joint_states", "joint_state_broadcaster/joint_states")],
    )
    spawn = Node(
        package="gazebo_ros", executable="spawn_entity.py", output="screen",
        arguments=["-entity", "assignment1_drivetrain", "-topic",
                   "/assignment1/robot_description", "-timeout", "60"],
    )
    controllers = [
        Node(
            package="controller_manager", executable="spawner", output="screen",
            arguments=[name, "--controller-manager", "/assignment1/controller_manager",
                       "--controller-manager-timeout", "60", "--switch-timeout", "30"],
        )
        for name in ["joint_state_broadcaster", "wheel_velocity_controller",
                     "steering_position_controller"]
    ]

    def after_spawn(event, context):
        """Fail the launch if Gazebo rejected the entity; otherwise start the controllers."""
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Assignment 1 entity spawn failed"))]
        return controllers

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false", description="Open the Gazebo GUI"),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)),
        gazebo, state_publisher, spawn,
    ])
