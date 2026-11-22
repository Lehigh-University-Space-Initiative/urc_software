"""Measure four existing drive joints in the suspended SDF-derived configuration."""
import math
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
from rclpy.node import Node
from cross_pkg_messages.msg import RoverDriveCommand2027
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import ListControllers

sys.path.insert(0, os.path.dirname(__file__))
from test_command_bridge_failure_launch import bridge_child_pid

PREFIX = "/assignment1_four_wheel"
FIELDS = ("front_left_velocity", "rear_left_velocity", "front_right_velocity", "rear_right_velocity")
JOINTS = tuple(f.replace("_velocity", "_wheel_joint") for f in FIELDS)

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    supervisor = ExecuteProcess(cmd=["ros2", "launch", "rover_sim_urc", "four_wheel.launch.py"], output="screen", additional_env={"PYTHONUNBUFFERED": "1"})
    return launch.LaunchDescription([supervisor, launch_testing.actions.ReadyToTest()]), {"supervisor": supervisor}

class Probe(Node):
    def __init__(self):
        super().__init__("four_wheel_feedback_probe")
        self.publisher = self.create_publisher(RoverDriveCommand2027, PREFIX + "/rover_drive_command_2027", 1)
        self.direct = self.create_publisher(RoverDriveCommand2027, PREFIX + "/validated_command", 1)
        self.sample = None
        self.sequence = 0
        self.outputs = []
        self.states = self.create_subscription(JointState, PREFIX + "/joint_state_broadcaster/joint_states", self.receive, 10)
        self.commands = self.create_subscription(Float64MultiArray, PREFIX + "/wheel_velocity_controller/commands", lambda m: self.outputs.append((time.monotonic(), list(m.data))), 10)

    def receive(self, msg):
        if not all(j in msg.name for j in JOINTS):
            return
        indices = [msg.name.index(j) for j in JOINTS]
        values = [(msg.position[i], msg.velocity[i]) for i in indices]
        if not all(math.isfinite(v) for pair in values for v in pair):
            raise AssertionError("Nonfinite measured feedback")
        self.sample = values
        self.sequence += 1

    def phase(self, target, duration=0.5, timeout=5, publisher=None, message=None):
        """Require sustained fresh joint feedback and signed travel, including inactive wheels."""
        start = time.monotonic(); last_send = 0; stable = None; previous = None
        travel = [0.0] * 4; count = 0; seq = self.sequence
        while time.monotonic() - start < timeout:
            now = time.monotonic()
            if publisher is not False and now - last_send >= 0.04:
                (publisher or self.publisher).publish(message or RoverDriveCommand2027(**dict(zip(FIELDS, target))))
                last_send = now
            rclpy.spin_once(self, timeout_sec=0.01)
            if seq == self.sequence: continue
            seq = self.sequence
            positions = [p for p, v in self.sample]
            if not all(abs(v - t) < 0.15 for (p, v), t in zip(self.sample, target)):
                stable = None; previous = None; travel = [0.0] * 4; count = 0
                continue
            if stable is None: stable = time.monotonic()
            if previous is not None:
                travel = [d + math.atan2(math.sin(p-q), math.cos(p-q)) for d,p,q in zip(travel,positions,previous)]
            previous = positions; count += 1
            if time.monotonic() - stable >= duration and count >= 20:
                for t,d in zip(target,travel):
                    if t == 0: assert abs(d) < 0.04, travel
                    else: assert d * math.copysign(1,t) > abs(t) * duration * 0.6, travel
                print(f"PASS four-wheel target={target}; measured={[round(v,6) for p,v in self.sample]}; travel={[round(v,6) for v in travel]}; samples={count}", flush=True)
                return time.monotonic() - start
        raise AssertionError(f"Feedback did not sustain {target}: {self.sample}")

    def expire(self):
        start = time.monotonic(); seq = self.sequence
        while time.monotonic() - start < 1.1:
            rclpy.spin_once(self, timeout_sec=0.01)
            if seq != self.sequence and all(abs(v) < 0.15 for p,v in self.sample):
                elapsed = time.monotonic() - start
                assert elapsed > 0.35, elapsed
                print(f"PASS four-wheel expiration={elapsed:.3f}s measured={[v for p,v in self.sample]}", flush=True)
                return
            seq = self.sequence
        raise AssertionError(f"Four-wheel expiration failed: {self.sample}")

class TestFourWheel(unittest.TestCase):
    def test_feedback_and_command_loss(self, proc_info, proc_output, supervisor):
        rclpy.init(); probe = Probe(); suspended = False; pid = None
        try:
            proc_info.assertWaitForStartup(process=supervisor, timeout=60)
            probe.phase([0.0]*4, timeout=60, publisher=False)
            # Require both controllers active, rather than interpreting static startup as success.
            client = probe.create_client(ListControllers, PREFIX + "/controller_manager/list_controllers")
            self.assertTrue(client.wait_for_service(timeout_sec=10))
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(probe, future, timeout_sec=10)
            self.assertIsNotNone(future.result())
            self.assertEqual({c.name:c.state for c in future.result().controller}, {"joint_state_broadcaster":"active", "wheel_velocity_controller":"active"})
            infos = probe.get_publishers_info_by_topic(PREFIX + "/wheel_velocity_controller/commands")
            self.assertEqual([i.node_name for i in infos], ["four_wheel_gate"])
            print("PASS four-wheel controllers active; gate sole output publisher", flush=True)
            for i in range(4):
                for sign in [1.0,-1.0]:
                    target = [0.0]*4; target[i] = sign*1.5
                    probe.phase(target)
            probe.phase([2.0]*4)
            probe.phase([-1.5]*4)
            probe.phase([0.0]*4)
            probe.phase([2.0]*4)
            probe.expire()  # bridge input silence
            probe.phase([0.0]*4, duration=1.0, publisher=False)
            probe.phase([2.0]*4)
            pid = bridge_child_pid(proc_info[supervisor].pid, "four_wheel_bridge.py")
            os.kill(pid, signal.SIGSTOP); suspended = True
            probe.expire()  # bridge watchdog cannot run
            start = time.monotonic()
            probe.phase([0.0]*4, duration=1.0, publisher=False)
            zeros = [v for t,v in probe.outputs if t > start + 0.1]
            self.assertGreaterEqual(len(zeros), 5)
            self.assertTrue(all(v == [0.0]*4 for v in zeros))
            # Direct gate validation cannot be masked by the suspended bridge.
            for field,value in [("front_left_velocity",float("nan")), ("rear_left_velocity",float("inf")), ("front_right_velocity",6.1), ("rear_right_velocity",-6.1), ("front_left_steering",0.1), ("front_right_steering",-0.1)]:
                probe.phase([2.0]*4, publisher=probe.direct)
                msg=RoverDriveCommand2027(**dict(zip(FIELDS,[2.0]*4)));setattr(msg,field,value)
                elapsed=probe.phase([0.0]*4, duration=0.3, publisher=probe.direct, message=msg)
                self.assertLess(elapsed,0.5)
                print(f"PASS four-wheel gate rejects {field}={value}; stop plus 0.3s hold={elapsed:.3f}s",flush=True)
            probe.phase([-1.5]*4,publisher=probe.direct)
            probe.expire() # valid gate envelope silence
            probe.phase([0.0]*4, duration=1.0,publisher=False)
            os.kill(pid,signal.SIGCONT);suspended=False
            probe.phase([-1.5]*4)
            for field,value in [("rear_right_velocity",float("nan")),("front_left_steering",0.1),("front_right_steering",-0.1)]:
                msg=RoverDriveCommand2027(**dict(zip(FIELDS,[2.0]*4)));setattr(msg,field,value)
                probe.phase([0.0]*4,message=msg)
            probe.phase([2.0]*4)
            gate=bridge_child_pid(proc_info[supervisor].pid,"four_wheel_gate.py")
            start=time.monotonic();os.kill(gate,signal.SIGKILL)
            proc_output.assertWaitFor(f"[pid {gate}, exit code -9",process=supervisor,timeout=5,stream="stdout")
            proc_output.assertWaitFor("[gzserver-1]: process has finished cleanly",process=supervisor,timeout=5,stream="stdout")
            proc_info.assertWaitForShutdown(process=supervisor,timeout=5)
            print(f"PASS four-wheel gate exit contained in {time.monotonic()-start:.3f}s",flush=True)
        finally:
            if suspended:
                try: os.kill(pid,signal.SIGCONT)
                except ProcessLookupError: pass
            probe.destroy_node()
            if rclpy.ok():rclpy.shutdown()
