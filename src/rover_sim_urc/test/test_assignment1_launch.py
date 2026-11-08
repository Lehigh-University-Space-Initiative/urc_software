"""Launch real Gazebo and assert the independent motion verifier exits successfully."""

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
    """Start the headless test assembly and a verifier with wall-clock motion deadlines."""
    simulation = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("rover_sim_urc"), "launch", "assignment1.launch.py")))
    verifier = Node(package="rover_sim_urc", executable="verify_assignment1.py", output="screen")
    return launch.LaunchDescription([
        simulation, verifier, launch_testing.actions.ReadyToTest(),
    ]), {"verifier": verifier}


class TestMotion(unittest.TestCase):
    def test_measured_motion(self, proc_info, verifier):
        """Wait for the probe's four stages; missing plugins or feedback cause failure."""
        proc_info.assertWaitForShutdown(process=verifier, timeout=150)
        launch_testing.asserts.assertExitCodes(proc_info, process=verifier)
