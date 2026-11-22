"""Start the existing two-joint test fixture with the simulation-only 2027 message bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Run independent bridge/gate processes and stop the fixture if either process exits."""
    share = get_package_share_directory("rover_sim_urc")
    bridge = Node(package="rover_sim_urc", executable="command_bridge_2027.py", output="screen")
    gate = Node(package="rover_sim_urc", executable="command_gate_2027.py", output="screen")
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(
                reason="2027 command bridge exited; stopping isolated simulation"))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=gate,
            on_exit=[EmitEvent(event=Shutdown(
                reason="2027 command gate exited; stopping isolated simulation"))],
        )),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "assignment1.launch.py"))),
        gate, bridge,
    ])
