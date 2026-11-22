"""Suspend only the real gate during measured motion; require bounded simulator shutdown."""
import os
import signal
import sys
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

sys.path.insert(0, os.path.dirname(__file__))
from test_four_wheel_launch import Probe, PREFIX
from test_command_bridge_failure_launch import bridge_child_pid


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    supervisor = ExecuteProcess(
        cmd=["ros2", "launch", "rover_sim_urc", "four_wheel.launch.py"],
        output="screen", additional_env={"PYTHONUNBUFFERED": "1"})
    return launch.LaunchDescription([
        supervisor, launch_testing.actions.ReadyToTest(),
    ]), {"supervisor": supervisor}


class TestGateHang(unittest.TestCase):
    def test_gate_sigstop_containment(self, proc_info, proc_output, supervisor):
        rclpy.init()
        probe = Probe()
        beats = []
        subscription = probe.create_subscription(
            UInt64, PREFIX + "/gate_progress", lambda msg: beats.append(msg.data), 1)
        gate_pid = None
        try:
            probe.phase([0.0] * 4, timeout=60, publisher=False)
            probe.phase([2.0] * 4)
            self.assertGreater(len(set(beats)), 5)
            monitor_pid = bridge_child_pid(proc_info[supervisor].pid, "gate_health_monitor.py")
            gate_pid = bridge_child_pid(proc_info[supervisor].pid, "four_wheel_gate.py")
            print(f"PASS health pre-failure: measured={[v for p,v in probe.sample]}; "
                  f"progress samples={len(set(beats))}; independent monitor PID={monitor_pid}", flush=True)
            started = time.monotonic()
            os.kill(gate_pid, signal.SIGSTOP)
            # Do not resume the gate: the production shutdown must contain the stopped process.
            proc_output.assertWaitFor("Gate progress deadline expired", process=supervisor,
                                     timeout=2, stream="stdout")
            detected = time.monotonic() - started
            proc_output.assertWaitFor("[gzserver-1]: process has finished cleanly",
                                     process=supervisor, timeout=3, stream="stdout")
            simulator_exit = time.monotonic() - started
            proc_info.assertWaitForShutdown(process=supervisor, timeout=5)
            total = time.monotonic() - started
            self.assertLess(detected, 1.0)
            self.assertLess(simulator_exit, 3.0)
            self.assertLess(total, 5.0)
            self.assertIn(proc_info[supervisor].returncode, (0, 1))
            print(f"PASS gate SIGSTOP: deadline detected={detected:.3f}s; "
                  f"clean Gazebo shutdown={simulator_exit:.3f}s; "
                  f"full launch shutdown={total:.3f}s; no wheel-braking claim", flush=True)
        finally:
            # Only resume on an unsuccessful test if launch cleanup has not yet killed it.
            if gate_pid is not None:
                try:
                    os.kill(gate_pid, signal.SIGCONT)
                except ProcessLookupError:
                    pass
            probe.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
