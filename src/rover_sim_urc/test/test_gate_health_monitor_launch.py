"""Test real monitor processes against healthy, absent and repeated stale progress.

All monitors use_sim_time=true without /clock; deadlines must still run.
"""
import time
import unittest

import launch
from launch.actions import ExecuteProcess
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from std_msgs.msg import UInt64


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    processes = {}
    for name in ["healthy", "silent", "stale"]:
        processes[name] = ExecuteProcess(
            cmd=["ros2", "run", "rover_sim_urc", "gate_health_monitor.py", "--ros-args",
                 "-p", "use_sim_time:=true", "-r", "__node:=health_test_" + name,
                 "-r", "/assignment1_four_wheel/gate_progress:=/health_test/" + name],
            output="screen", additional_env={"RCUTILS_LOGGING_USE_STDOUT": "1"})
    return launch.LaunchDescription([
        *processes.values(), launch_testing.actions.ReadyToTest(),
    ]), {"processes": processes}


class TestMonitor(unittest.TestCase):
    def test_deadlines_without_simulation_clock(self, proc_info, proc_output, processes):
        rclpy.init()
        node = rclpy.create_node("health_test_input")
        healthy = node.create_publisher(UInt64, "/health_test/healthy", 1)
        stale = node.create_publisher(UInt64, "/health_test/stale", 1)
        stale_value = None
        first_stale = None
        exits = {}
        started = time.monotonic()
        try:
            # Keep one monitor healthy past the three-second silent-start deadline.
            while time.monotonic() - started < 4.0:
                now = time.monotonic_ns()
                healthy.publish(UInt64(data=now))
                if stale.get_subscription_count() > 0:
                    if stale_value is None:
                        stale_value = now
                        first_stale = time.monotonic()
                    stale.publish(UInt64(data=stale_value))
                rclpy.spin_once(node, timeout_sec=0.04)
                for name in ["silent", "stale"]:
                    if name not in exits and getattr(proc_info[processes[name]], "returncode", None) is not None:
                        exits[name] = time.monotonic()
            self.assertIsNotNone(first_stale, "stale test never discovered monitor")
            for name in ["silent", "stale"]:
                proc_info.assertWaitForShutdown(process=processes[name], timeout=2)
                self.assertEqual(proc_info[processes[name]].returncode, 2)
                proc_output.assertWaitFor("Gate progress deadline expired", process=processes[name], timeout=1, stream="stdout")
            self.assertEqual(set(exits), {"silent", "stale"})
            self.assertGreater(exits["stale"] - first_stale, 0.35)
            self.assertLess(exits["stale"] - first_stale, 1.5)
            self.assertIsNone(getattr(proc_info[processes["healthy"]], "returncode", None))
            stopped = time.monotonic()
            proc_info.assertWaitForShutdown(process=processes["healthy"] , timeout=2)
            elapsed = time.monotonic() - stopped
            self.assertLess(elapsed, 1.0)
            self.assertEqual(proc_info[processes["healthy"]].returncode, 2)
            print(f"PASS health monitor: fresh progress sustained 4s without /clock; "
                  f"silent startup and repeated stale values exited 2; "
                  f"silent exit from test start={exits['silent'] - started:.3f}s; "
                  f"stale first-send to exit={exits['stale'] - first_stale:.3f}s; "
                  f"fresh-input cessation to process exit={elapsed:.3f}s", flush=True)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
