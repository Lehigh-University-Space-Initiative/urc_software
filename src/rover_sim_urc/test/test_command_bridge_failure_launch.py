"""Kill only the simulated command bridge during measured motion and require Gazebo shutdown.

This tests launch supervision after abrupt process death, when the bridge cannot
send a final stop command. It does not represent a physical hardware stop policy.
"""

import os
import signal
import sys
import time
import unittest

from ament_index_python.packages import get_package_prefix
import launch
from launch.actions import ExecuteProcess
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Exercise the production bridge launch with its actual process-exit supervision."""
    # Isolating the supervised LaunchService so its intended shutdown does not stop the test runner
    supervisor = ExecuteProcess(
        cmd=["ros2", "launch", "rover_sim_urc", "assignment1_command_2027.launch.py"],
        name="bridge_supervisor", output="screen", additional_env={"PYTHONUNBUFFERED": "1"},
    )
    return launch.LaunchDescription([
        supervisor, launch_testing.actions.ReadyToTest(),
    ]), {"supervisor": supervisor}


def bridge_child_pid(supervisor_pid, script_name="command_bridge_2027.py"):
    """Find exactly one named executable descending from this test's supervisor on Linux."""
    script = os.fsencode(os.path.join(
        get_package_prefix("rover_sim_urc"), "lib", "rover_sim_urc", script_name))
    matches = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open(f"/proc/{name}/cmdline", "rb") as f:
                if script not in f.read().split(b"\0"):
                    continue
            pid = int(name)
            ancestors = set()
            while pid > 1 and pid not in ancestors:
                if pid == supervisor_pid:
                    matches.append(int(name))
                    break
                ancestors.add(pid)
                with open(f"/proc/{pid}/status") as f:
                    pid = next(int(line.split()[1]) for line in f if line.startswith("PPid:"))
        except (FileNotFoundError, ProcessLookupError):
            continue
    if len(matches) != 1:
        raise RuntimeError(f"Expected one bridge child of PID {supervisor_pid}, found {matches}")
    return matches[0]


class TestBridgeFailure(unittest.TestCase):
    def test_abrupt_bridge_death_stops_simulation(self, proc_info, proc_output, supervisor):
        """Require real motion before SIGKILL, then a clean Gazebo exit within five seconds."""
        # Reusing feedback assertions and sending only typed commands through the bridge
        sys.path.insert(0, os.path.join(get_package_prefix("rover_sim_urc"), "lib", "rover_sim_urc"))
        from verify_command_bridge_2027 import MessageProbe

        rclpy.init()
        probe = MessageProbe()
        try:
            proc_info.assertWaitForStartup(process=supervisor, timeout=60)
            probe.phase(0.0, 0.0, timeout=60.0)
            probe.phase(2.0, 0.3)
            position, velocity, steering = probe.sample
            print(f"PASS: before bridge failure, wheel={velocity:.3f} rad/s, "
                  f"steering={steering:.3f} rad, wheel position={position:.3f} rad", flush=True)

            # Targeting exactly the bridge launched by this test, inside its test container
            pid = bridge_child_pid(proc_info[supervisor].pid)
            started = time.monotonic()
            os.kill(pid, signal.SIGKILL)
            proc_output.assertWaitFor("exit code -9", process=supervisor, timeout=5,
                                     stream="stdout")
            proc_output.assertWaitFor("[gzserver-1]: process has finished cleanly",
                                     process=supervisor, timeout=5, stream="stdout")
            proc_info.assertWaitForShutdown(process=supervisor, timeout=5)
            elapsed = time.monotonic() - started
            # ros2 launch may report the intentionally killed child as a nonzero launch result
            self.assertIn(proc_info[supervisor].returncode, (0, 1))
            self.assertLess(elapsed, 5.0)
            print(f"PASS: bridge SIGKILL caused clean Gazebo shutdown in {elapsed:.3f}s", flush=True)
        finally:
            # Sending no cleanup command: containment must come from the launch supervisor
            probe.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
