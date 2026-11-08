#!/usr/bin/env python3
"""Exercise the generated 2027 ROS message against measured Gazebo joint feedback.

Reuses the existing motion probe's feedback checks but overrides its command path.
No direct-controller publishers are created: all commands go through
/assignment1/rover_drive_command_2027. Tests run on the simulation PC only.
"""

import time

from cross_pkg_messages.msg import RoverDriveCommand2027
import rclpy

from verify_assignment1 import MotionProbe


class MessageProbe(MotionProbe):
    """Publish typed 2027 commands while retaining the proven independent motion assertions."""

    def __init__(self):
        super().__init__(direct_commands=False)
        self.enabled = True
        self.publisher = self.create_publisher(
            RoverDriveCommand2027, "/assignment1/rover_drive_command_2027", 10)

    def command(self, velocity, angle):
        """Send only front-left fields; disabling publication exercises the watchdog."""
        if self.enabled:
            self.publisher.publish(RoverDriveCommand2027(
                front_left_velocity=velocity, front_left_steering=angle))

    def rejected_phase(self, field, value):
        """Start moving, send an invalid field once, and require stop before the timeout."""
        self.enabled = True
        self.phase(2.0, 0.3)
        invalid = RoverDriveCommand2027(front_left_velocity=2.0, front_left_steering=0.3)
        setattr(invalid, field, value)
        self.publisher.publish(invalid)
        self.enabled = False
        latency = self.wait_for_stop(0.4)
        self.phase(0.0, 0.0)
        print(f"PASS: invalid {field}={value} stopped and centered measured joints "
              f"in {latency:.3f}s (before 0.5s watchdog)", flush=True)

    def wait_for_stop(self, timeout):
        """Measure time to fresh stopped feedback, separately from the sustained rest check."""
        started = time.monotonic()
        sequence = self.sequence
        while time.monotonic() - started < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.sequence != sequence:
                sequence = self.sequence
                _, velocity, angle = self.sample
                if abs(velocity) < 0.15 and abs(angle) < 0.04:
                    return time.monotonic() - started
        raise RuntimeError(f"No measured stop within {timeout}s; last feedback={self.sample}")


def main():
    """Check forward/reverse motion, timeout, unsupported fields, nonfinite values, and limits."""
    rclpy.init()
    probe = MessageProbe()
    try:
        probe.phase(0.0, 0.0, timeout=60.0)
        probe.phase(2.0, 0.3)
        probe.phase(-1.5, -0.25)
        probe.enabled = False
        latency = probe.wait_for_stop(2.0)
        if latency < 0.35:
            raise RuntimeError(f"Unexpectedly early watchdog stop: {latency}s")
        probe.phase(0.0, 0.0)
        print(f"PASS: command silence triggered measured stop and centering in {latency:.3f}s",
              flush=True)
        for field, value in [
            ("rear_left_velocity", 1.0), ("front_right_velocity", 1.0),
            ("rear_right_velocity", 1.0), ("front_right_steering", 0.1),
            ("front_left_velocity", float("nan")),
            ("front_left_steering", float("inf")),
            ("front_left_velocity", 6.1), ("front_left_steering", 0.61),
        ]:
            probe.rejected_phase(field, value)
        probe.enabled = True
        probe.phase(-1.5, -0.25)
        probe.phase(0.0, 0.0)
        print("PASS: 2027 message bridge motion and rejection/timeout checks verified", flush=True)
    finally:
        probe.enabled = True
        probe.command(0.0, 0.0)
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
