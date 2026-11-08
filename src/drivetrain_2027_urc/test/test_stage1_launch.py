"""Real ROS interfaces with explicitly synthetic joint feedback; never physical motor control."""
import json
import math
import time
import unittest
import os
import sys
from ament_index_python.packages import get_package_prefix

import launch
from launch_ros.actions import Node
import launch_testing.actions
import pytest
import rclpy
from rclpy.clock import Clock, ClockType
from cross_pkg_messages.msg import RoverDriveCommand2027
from cross_pkg_messages.srv import SetDriveMode2027
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

PREFIX = "/dcs_stage1"
FIELDS = ("front_left_velocity", "rear_left_velocity", "front_right_velocity", "rear_right_velocity")
JOINTS = [s.replace("_velocity", "_wheel_joint") for s in FIELDS] + [
    "front_left_steering_joint", "front_right_steering_joint"]


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        Node(package="drivetrain_2027_urc", executable="dcs_stage1.py", output="screen"),
        launch_testing.actions.ReadyToTest(),
    ])


class TestStage1(unittest.TestCase):
    def test_interfaces_and_safety(self):
        rclpy.init()
        probe = rclpy.create_node("stage1_mock_test")
        clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        status = {}
        outputs = []
        manual = probe.create_publisher(RoverDriveCommand2027, PREFIX + "/manual_command", 1)
        autonomy = probe.create_publisher(TwistStamped, PREFIX + "/autonomy_cmd_vel", 1)
        feedback = probe.create_publisher(JointState, PREFIX + "/mock_feedback", 1)
        emergency = probe.create_publisher(Bool, PREFIX + "/emergency_stop", 1)
        status_sub = probe.create_subscription(String, PREFIX + "/status", lambda m: status.update(json.loads(m.data)), 10)
        output_sub = probe.create_subscription(RoverDriveCommand2027, PREFIX + "/mock_command", lambda m: outputs.append(m), 10)
        mode = probe.create_client(SetDriveMode2027, PREFIX + "/set_mode")
        reset = probe.create_client(Trigger, PREFIX + "/reset_fault")
        mock_angles = [0.0, 0.0]

        def spin(duration, feedback_enabled=True, manual_values=None, auto_values=None):
            started = time.monotonic()
            while time.monotonic() - started < duration:
                if feedback_enabled:
                    msg = JointState(name=JOINTS, position=[0.0] * 4 + mock_angles, velocity=[0.0] * 6)
                    msg.header.stamp = clock.now().to_msg()
                    feedback.publish(msg)
                if manual_values is not None:
                    msg = RoverDriveCommand2027(**dict(zip(FIELDS, manual_values[:4])),
                        front_left_steering=manual_values[4], front_right_steering=manual_values[5])
                    msg.header.stamp = clock.now().to_msg()
                    manual.publish(msg)
                if auto_values is not None:
                    msg = TwistStamped()
                    msg.header.stamp = clock.now().to_msg()
                    msg.twist.linear.x, msg.twist.angular.z = auto_values
                    autonomy.publish(msg)
                rclpy.spin_once(probe, timeout_sec=0.02)

        def wait(predicate, **kwargs):
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                spin(0.03, **kwargs)
                if predicate():
                    return
            self.fail(f"ROS condition timed out: status={status}")

        def call(client, request, feedback_enabled=True):
            self.assertTrue(client.wait_for_service(timeout_sec=10))
            future = client.call_async(request)
            deadline = time.monotonic() + 3
            while not future.done() and time.monotonic() < deadline:
                spin(0.02, feedback_enabled=feedback_enabled)
            self.assertTrue(future.done())
            return future.result()

        def wheels():
            return tuple(getattr(outputs[-1], f) for f in FIELDS) if outputs else None

        def fault(name, **kwargs):
            wait(lambda: status.get("fault") == name and status.get("mode") == "DISABLED" and wheels() == (0.0,) * 4, **kwargs)
            self.assertFalse(status["physical_outputs_enabled"])
            print("PASS Stage 1 ROS fault:", name, "mock wheels=", wheels(), flush=True)

        def clear():
            wait(lambda: status.get("feedback_fresh"))
            self.assertTrue(call(reset, Trigger.Request()).success)
            wait(lambda: status.get("fault") == "")

        try:
            sys.path.insert(0, os.path.join(get_package_prefix("drivetrain_2027_urc"), "lib", "drivetrain_2027_urc"))
            from dcs_stage1 import stamp_ns
            malformed = RoverDriveCommand2027()
            malformed.header.stamp.sec = 1
            malformed.header.stamp.nanosec = 1_000_000_000
            self.assertEqual(stamp_ns(malformed.header), 0)
            wait(lambda: status.get("mode") == "DISABLED", feedback_enabled=False)
            self.assertFalse(call(mode, SetDriveMode2027.Request(mode=1), feedback_enabled=False).accepted)
            wait(lambda: status.get("feedback_fresh"))
            self.assertTrue(call(mode, SetDriveMode2027.Request(mode=1)).accepted)
            wait(lambda: wheels() == (1.0, -1.0, 2.0, -2.0), manual_values=(1.0, -1.0, 2.0, -2.0, 0.2, -0.2))
            print("PASS Stage 1 ROS manual observed mock output", wheels(), flush=True)
            started = time.monotonic()
            fault("COMMAND_TIMEOUT")
            print(f"PASS Stage 1 ROS command loss to observed disabled zero={time.monotonic()-started:.3f}s", flush=True)
            spin(0.2, manual_values=(1.0,) * 4 + (0.0, 0.0))
            self.assertEqual(status["mode"], "DISABLED")
            clear()
            self.assertTrue(call(mode, SetDriveMode2027.Request(mode=1)).accepted)
            fault("INVALID_OR_STALE_MANUAL", manual_values=(float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0))
            clear()
            self.assertTrue(call(mode, SetDriveMode2027.Request(mode=1)).accepted)
            msg = RoverDriveCommand2027(front_left_velocity=1.0)
            msg.header.stamp = clock.now().to_msg()
            msg.header.stamp.sec -= 1
            manual.publish(msg)
            fault("INVALID_OR_STALE_MANUAL")
            clear()
            mock_angles[:] = [0.3, -0.2]
            spin(0.1)
            self.assertFalse(call(mode, SetDriveMode2027.Request(mode=2)).accepted)
            self.assertEqual(status["mode"], "DISABLED")
            self.assertTrue(call(mode, SetDriveMode2027.Request(mode=1)).accepted)
            wait(lambda: outputs[-1].front_left_steering == 0.0, manual_values=(0.0,) * 6)
            # Centering must come from a new feedback sample, not the requested angle.
            self.assertFalse(call(mode, SetDriveMode2027.Request(mode=2)).accepted)
            mock_angles[:] = [0.0, 0.0]
            spin(0.1)
            self.assertTrue(call(mode, SetDriveMode2027.Request(mode=2)).accepted)
            wait(lambda: wheels() == (1.0,) * 4, auto_values=(0.108, 0.0))
            print("PASS Stage 1 ROS autonomy v=0.108m/s observed mock output", wheels(), flush=True)
            wrong_frame = TwistStamped()
            wrong_frame.header.stamp = clock.now().to_msg()
            wrong_frame.header.frame_id = "map"
            wrong_frame.twist.linear.x = 0.108
            autonomy.publish(wrong_frame)
            fault("INVALID_OR_STALE_AUTONOMY")
            clear()
            self.assertTrue(call(mode, SetDriveMode2027.Request(mode=2)).accepted)
            wait(lambda: wheels() is not None and abs(wheels()[0] + 2.3208333333333333) < 1e-9,
                 auto_values=(0.0, 1.0), manual_values=(4.0,) * 4 + (0.0, 0.0))
            print("PASS Stage 1 ROS yaw=+1rad/s observed mock output", wheels(), "inactive manual ignored", flush=True)
            mock_angles[:] = [0.03, 0.0]
            fault("STEERING_NOT_CENTERED")
            mock_angles[:] = [0.0, 0.0]
            spin(0.1)
            clear()
            self.assertTrue(call(mode, SetDriveMode2027.Request(mode=2)).accepted)
            wait(lambda: wheels() == (1.0,) * 4, auto_values=(0.108, 0.0))
            fault("FEEDBACK_TIMEOUT", feedback_enabled=False, auto_values=(0.108, 0.0))
            self.assertFalse(call(reset, Trigger.Request(), feedback_enabled=False).success)
            spin(0.1)
            clear()
            self.assertTrue(call(mode, SetDriveMode2027.Request(mode=1)).accepted)
            emergency.publish(Bool(data=True))
            fault("EMERGENCY_STOP")
            self.assertFalse(call(reset, Trigger.Request()).success)
            emergency.publish(Bool(data=False))
            spin(0.1)
            self.assertEqual(status["fault"], "EMERGENCY_STOP")
            clear()
            self.assertEqual(status["mode"], "DISABLED")
            publishers = probe.get_publisher_names_and_types_by_node("dcs_stage1_mock_arbiter", "/")
            allowed = {PREFIX + "/mock_command", PREFIX + "/status", "/rosout", "/parameter_events"}
            self.assertTrue({name for name, types in publishers} <= allowed, publishers)
            self.assertTrue(status["feedback_is_mock"])
            print("PASS Stage 1 ROS: explicit reset leaves DISABLED; only mock/status/ROS-infrastructure publishers", flush=True)
        finally:
            probe.destroy_node()
            rclpy.try_shutdown()
