"""Verify the gate's startup zero output in Gazebo with no envelope publisher or bridge."""

import os
import sys
import time
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing.actions
import pytest
import rclpy
from std_msgs.msg import Float64MultiArray


@pytest.mark.launch_test
def generate_test_description():
    """Start only the unchanged fixture and the real gate; no bridge can supply zero envelopes."""
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("rover_sim_urc"), "launch", "assignment1.launch.py"))),
        Node(package="rover_sim_urc", executable="command_gate_2027.py", output="screen"),
        launch_testing.actions.ReadyToTest(),
    ])


class TestGateStartup(unittest.TestCase):
    def test_startup_without_envelopes(self):
        """Require fresh stopped feedback and repeated zero output without any envelope writer."""
        sys.path.insert(0, os.path.join(get_package_prefix("rover_sim_urc"), "lib", "rover_sim_urc"))
        from verify_command_bridge_2027 import MessageProbe

        rclpy.init()
        probe = MessageProbe()
        probe.enabled = False
        outputs = [[], []]
        subscriptions = [probe.create_subscription(
            Float64MultiArray, f"/assignment1/{controller}/commands",
            lambda msg, i=i: outputs[i].append(list(msg.data)), 1)
            for i, controller in enumerate(["wheel_velocity_controller", "steering_position_controller"])]
        try:
            probe.phase(0.0, 0.0, timeout=60.0)
            self.assertEqual(probe.get_publishers_info_by_topic(
                "/assignment1/validated_drive_command_2027"), [])
            for values in outputs:
                values.clear()
            started = time.monotonic()
            sequence = probe.sequence
            samples = 0
            while time.monotonic() - started < 1.0:
                rclpy.spin_once(probe, timeout_sec=0.01)
                if probe.sequence != sequence:
                    sequence = probe.sequence
                    samples += 1
                    self.assertLess(abs(probe.sample[1]), 0.15)
                    self.assertLess(abs(probe.sample[2]), 0.04)
            self.assertGreater(samples, 20)
            for values in outputs:
                self.assertGreaterEqual(len(values), 10)
                self.assertTrue(all(value == [0.0] for value in values))
            self.assertEqual(probe.get_publishers_info_by_topic(
                "/assignment1/validated_drive_command_2027"), [])
            print(f"PASS: gate startup with zero envelope publishers; {samples} stopped samples "
                  f"over 1.0s, repeated zero outputs={[len(values) for values in outputs]}", flush=True)
        finally:
            probe.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
