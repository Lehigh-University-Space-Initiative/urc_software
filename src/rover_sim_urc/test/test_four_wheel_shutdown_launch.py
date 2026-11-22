"""Verify real idle/moving SIGINT and monitor-triggered shutdown without cleanup publication errors."""
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import unittest

import launch
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from controller_manager_msgs.srv import ListControllers

sys.path.insert(0, os.path.dirname(__file__))
from test_four_wheel_launch import Probe, PREFIX
from test_command_bridge_failure_launch import bridge_child_pid


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])


class TestShutdown(unittest.TestCase):
    def test_shutdown_paths(self):
        # Each scenario gets its own real production launch, serially in this test container.
        root = Path(os.environ["ROS_LOG_DIR"]).parent / "cleanup-scenarios"
        root.mkdir()
        for mode in ["idle", "moving", "monitor"]:
            path = root / (mode + ".txt")
            with path.open("x") as output:
                child = subprocess.Popen(
                    ["ros2", "launch", "rover_sim_urc", "four_wheel.launch.py"],
                    stdout=output, stderr=subprocess.STDOUT, start_new_session=True,
                    env={**os.environ, "PYTHONUNBUFFERED": "1"})
                rclpy.init()
                probe = Probe()
                try:
                    probe.phase([0.0] * 4, timeout=60, publisher=False)
                    client = probe.create_client(ListControllers, PREFIX + "/controller_manager/list_controllers")
                    self.assertTrue(client.wait_for_service(timeout_sec=10))
                    future = client.call_async(ListControllers.Request())
                    rclpy.spin_until_future_complete(probe, future, timeout_sec=10)
                    self.assertIsNotNone(future.result())
                    self.assertEqual({c.name: c.state for c in future.result().controller},
                                     {"joint_state_broadcaster": "active", "wheel_velocity_controller": "active"})
                    if mode != "idle":
                        probe.phase([2.0] * 4)
                    print(f"PASS cleanup scenario {mode}: before shutdown measured={[v for p,v in probe.sample]}", flush=True)
                    started = time.monotonic()
                    if mode == "monitor":
                        gate = bridge_child_pid(child.pid, "four_wheel_gate.py")
                        os.kill(gate, signal.SIGSTOP)
                    else:
                        child.send_signal(signal.SIGINT)
                    result = child.wait(timeout=5)
                    elapsed = time.monotonic() - started
                    text = path.read_text(errors="replace")
                    self.assertIn(result, (0, 1) if mode == "monitor" else (0,))
                    self.assertNotIn("Traceback", text)
                    self.assertNotIn("publisher's context is invalid", text)
                    self.assertNotIn("shutdown already called", text)
                    self.assertIn("[gzserver-1]: process has finished cleanly", text)
                    self.assertRegex(text, r"\[four_wheel_bridge.py-\d+\]: process has finished cleanly")
                    if mode == "monitor":
                        self.assertIn("Gate progress deadline expired", text)
                        self.assertRegex(text, r"four_wheel_gate.py.*exit code -9")
                    else:
                        self.assertRegex(text, r"\[four_wheel_gate.py-\d+\]: process has finished cleanly")
                    print(f"PASS cleanup scenario {mode}: full shutdown={elapsed:.3f}s; "
                          f"bridge clean; no traceback; Gazebo clean; result={result}", flush=True)
                finally:
                    # Only this test's new process group is eligible for fallback cleanup.
                    if child.poll() is None:
                        child.send_signal(signal.SIGINT)
                        try:
                            child.wait(timeout=15)
                        except subprocess.TimeoutExpired:
                            os.killpg(child.pid, signal.SIGKILL)
                            child.wait(timeout=5)
                    probe.destroy_node()
                    rclpy.try_shutdown()
