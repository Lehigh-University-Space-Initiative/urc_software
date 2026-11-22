"""Measure independent gate expiration, rejection, recovery, sole output ownership, and exit containment.

Uses the real production launch in a child LaunchService, with SIGSTOP applied only
to its bridge descendant. Invalid envelopes are injected downstream of that bridge.
"""

import math
import os
import signal
import sys
import time
import unittest

from ament_index_python.packages import get_package_prefix
from cross_pkg_messages.msg import RoverDriveCommand2027
import launch
from launch.actions import ExecuteProcess
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, os.path.dirname(__file__))
from test_command_bridge_failure_launch import bridge_child_pid


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Observe the actual bridge-and-gate launch without propagating its shutdown to the runner."""
    supervisor = ExecuteProcess(
        cmd=["ros2", "launch", "rover_sim_urc", "assignment1_command_2027.launch.py"],
        name="gate_supervisor", output="screen", additional_env={"PYTHONUNBUFFERED": "1"},
    )
    return launch.LaunchDescription([
        supervisor, launch_testing.actions.ReadyToTest(),
    ]), {"supervisor": supervisor}


class TestCommandGate(unittest.TestCase):
    def test_gate_feedback_and_failures(self, proc_info, proc_output, supervisor):
        """Require measured stops while the bridge is suspended and clean shutdown on gate death."""
        sys.path.insert(0, os.path.join(get_package_prefix("rover_sim_urc"), "lib", "rover_sim_urc"))
        from verify_command_bridge_2027 import MessageProbe

        rclpy.init()
        probe = MessageProbe()
        probe.enabled = False
        bridge_pid = None
        suspended = False
        outputs = {"wheel": [], "steering": []}
        subscriptions = []
        for key, controller in [("wheel", "wheel_velocity_controller"),
                                ("steering", "steering_position_controller")]:
            subscriptions.append(probe.create_subscription(
                Float64MultiArray, f"/assignment1/{controller}/commands",
                lambda msg, key=key: outputs[key].append((time.monotonic(), list(msg.data))), 1))

        def stopped_interval(duration, invalid=None):
            """Require fresh stopped feedback, negligible travel, and repeated zero controller output."""
            started = time.monotonic()
            sequence = probe.sequence
            previous = None
            travel = 0.0
            samples = 0
            while time.monotonic() - started < duration:
                if invalid is not None:
                    probe.publisher.publish(invalid)
                rclpy.spin_once(probe, timeout_sec=0.01)
                if probe.sequence == sequence:
                    continue
                sequence = probe.sequence
                position, velocity, angle = probe.sample
                self.assertLess(abs(velocity), 0.15)
                self.assertLess(abs(angle), 0.04)
                if previous is not None:
                    travel += math.atan2(math.sin(position - previous), math.cos(position - previous))
                previous = position
                samples += 1
            self.assertGreater(samples, 20)
            self.assertLess(abs(travel), 0.04)
            counts = {}
            for key in outputs:
                messages = [values for when, values in outputs[key] if when > started + 0.1]
                self.assertGreaterEqual(len(messages), 5)
                self.assertTrue(all(values == [0.0] for values in messages))
                counts[key] = len(messages)
            print(f"PASS: stopped for {duration:.2f}s, samples={samples}, travel={travel:.6f} rad, "
                  f"repeated zero outputs={counts}", flush=True)

        try:
            proc_info.assertWaitForStartup(process=supervisor, timeout=60)
            # No upstream commands have been sent; the gate must establish startup zero output
            probe.phase(0.0, 0.0, timeout=60.0)
            stopped_interval(0.8)
            for controller in ["wheel_velocity_controller", "steering_position_controller"]:
                infos = probe.get_publishers_info_by_topic(f"/assignment1/{controller}/commands")
                self.assertEqual([info.node_name for info in infos], ["assignment1_command_gate_2027"])
            print("PASS: gate is the sole publisher on both controller-command topics", flush=True)

            probe.enabled = True
            probe.phase(2.0, 0.3)
            bridge_pid = bridge_child_pid(proc_info[supervisor].pid)
            probe.enabled = False
            os.kill(bridge_pid, signal.SIGSTOP)
            suspended = True
            latency = probe.wait_for_stop(1.0)
            self.assertGreater(latency, 0.35)
            print(f"PASS: bridge SIGSTOP; independent measured stop/centering in {latency:.3f}s, "
                  f"feedback={probe.sample}", flush=True)
            stopped_interval(1.0)
            os.kill(proc_info[supervisor].pid, 0)
            with open(f"/proc/{bridge_pid}/status") as f:
                self.assertTrue(any(line.startswith("State:") and "T" in line for line in f))

            # Testing validation at the gate input while the bridge cannot sanitize these envelopes
            upstream_publisher = probe.publisher
            probe.publisher = probe.create_publisher(
                RoverDriveCommand2027, "/assignment1/validated_drive_command_2027", 1)
            for field, value in [
                ("rear_left_velocity", 1.0), ("front_right_velocity", 1.0),
                ("rear_right_velocity", 1.0), ("front_right_steering", 0.1),
                ("front_left_velocity", float("nan")),
                ("front_left_steering", float("inf")),
                ("front_left_velocity", 6.1), ("front_left_steering", 0.61),
                ("front_left_velocity", -6.1), ("front_left_steering", -0.61),
            ]:
                probe.rejected_phase(field, value)
            invalid = RoverDriveCommand2027(front_left_velocity=float("nan"))
            stopped_interval(1.0, invalid=invalid)

            # Expiring a directly injected valid envelope proves the gate owns its own lease
            probe.enabled = True
            probe.phase(-1.5, -0.25)
            probe.enabled = False
            latency = probe.wait_for_stop(1.0)
            self.assertGreater(latency, 0.35)
            print(f"PASS: gate-input silence expired independently in {latency:.3f}s", flush=True)
            stopped_interval(1.0)

            probe.publisher = upstream_publisher
            os.kill(bridge_pid, signal.SIGCONT)
            suspended = False
            probe.enabled = True
            probe.phase(-1.5, -0.25)
            print("PASS: fresh upstream commands recovered motion through bridge and gate", flush=True)
            probe.phase(0.0, 0.0)
            probe.phase(2.0, 0.3)
            gate_pid = bridge_child_pid(proc_info[supervisor].pid, "command_gate_2027.py")
            started = time.monotonic()
            os.kill(gate_pid, signal.SIGKILL)
            proc_output.assertWaitFor(f"[pid {gate_pid}, exit code -9",
                                     process=supervisor, timeout=5, stream="stdout")
            proc_output.assertWaitFor("[gzserver-1]: process has finished cleanly",
                                     process=supervisor, timeout=5, stream="stdout")
            proc_info.assertWaitForShutdown(process=supervisor, timeout=5)
            elapsed = time.monotonic() - started
            self.assertLess(elapsed, 5.0)
            self.assertIn(proc_info[supervisor].returncode, (0, 1))
            print(f"PASS: gate SIGKILL caused clean Gazebo shutdown in {elapsed:.3f}s", flush=True)
        finally:
            if suspended:
                try:
                    os.kill(bridge_pid, signal.SIGCONT)
                except ProcessLookupError:
                    pass
            probe.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
