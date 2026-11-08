"""Launch real Gazebo with the 2027 message bridge and assert measured motion checks pass."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest


@pytest.mark.launch_test
def generate_test_description():
    """Include the standalone bridge launch and a verifier using only typed commands."""
    simulation = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("rover_sim_urc"), "launch", "assignment1_command_2027.launch.py")))
    verifier = Node(
        package="rover_sim_urc", executable="verify_command_bridge_2027.py", output="screen")
    return launch.LaunchDescription([
        simulation, verifier, launch_testing.actions.ReadyToTest(),
    ]), {"verifier": verifier}


class TestCommandBridge(unittest.TestCase):
    def test_measured_message_commands(self, proc_info, verifier):
        """Fail if the bridge cannot command motion or stop on rejected/expired commands."""
        proc_info.assertWaitForShutdown(process=verifier, timeout=150)
        launch_testing.asserts.assertExitCodes(proc_info, process=verifier)
