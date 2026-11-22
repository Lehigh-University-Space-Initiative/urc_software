"""Force real context invalidation around the shared bridge/gate cleanup helper."""
import os
import sys
import unittest
from unittest.mock import patch

from ament_index_python.packages import get_package_prefix
import launch
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.executors import ExternalShutdownException


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])


class TestCleanup(unittest.TestCase):
    def test_shared_cleanup(self):
        sys.path.insert(0, os.path.join(get_package_prefix("rover_sim_urc"), "lib", "rover_sim_urc"))
        from four_wheel_bridge import FourWheelBridge, cleanup, run
        from four_wheel_gate import FourWheelGate

        for cls in [FourWheelBridge, FourWheelGate]:
            # Actual ROS objects/context; intercept spin only to force the shutdown boundary.
            nodes = []
            def make_node():
                node = cls()
                nodes.append(node)
                return node

            def invalidate(node):
                node.context.try_shutdown()
                raise ExternalShutdownException()

            with patch("four_wheel_bridge.rclpy.spin", side_effect=invalidate):
                with patch.object(cls, "tick", side_effect=AssertionError("Cleanup published")):
                    run(make_node)
            self.assertFalse(nodes[0].context.ok())
            cleanup(nodes[0])  # Repeated destruction/context cleanup must be harmless.
            print(f"PASS cleanup {cls.__name__}: forced context invalidation, no teardown tick, repeated cleanup", flush=True)

            # A non-shutdown failure must propagate, rather than being hidden by cleanup.
            with patch("four_wheel_bridge.rclpy.spin", side_effect=RuntimeError("live failure")):
                with self.assertRaisesRegex(RuntimeError, "live failure"):
                    run(cls)
            self.assertFalse(rclpy.ok())

            # Context cleanup still runs if destruction itself fails; do not suppress that failure.
            rclpy.init()
            node = cls()
            try:
                with patch.object(node, "destroy_node", side_effect=RuntimeError("destroy failure")):
                    with self.assertRaisesRegex(RuntimeError, "destroy failure"):
                        cleanup(node)
                self.assertFalse(node.context.ok())
            finally:
                cleanup(node)
            print(f"PASS cleanup {cls.__name__}: live/destruction errors propagate; context still shuts down", flush=True)

        def failed_constructor():
            raise RuntimeError("constructor failure")
        with self.assertRaisesRegex(RuntimeError, "constructor failure"):
            run(failed_constructor)
        self.assertFalse(rclpy.ok())
        print("PASS cleanup: constructor failure also cleans initialized context", flush=True)
