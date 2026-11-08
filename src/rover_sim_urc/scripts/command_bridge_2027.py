#!/usr/bin/env python3
"""Map the 2027 message to the simulated front-left assembly, without kinematics.

Started on the development PC by assignment1_command_2027.launch.py.
Subscribes to /assignment1/rover_drive_command_2027 (RoverDriveCommand2027).
Publishes validated RoverDriveCommand2027 envelopes to /assignment1/validated_drive_command_2027.
Unsupported actuators must be zero; malformed, out-of-range, or expired commands stop the stand.
All limits here describe the existing test fixture, not physical hardware specifications.
"""

import math
import time

from cross_pkg_messages.msg import RoverDriveCommand2027
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


class CommandBridge(Node):
    """Forward only the two supported setpoints and stop on receipt-time expiration."""

    def __init__(self):
        super().__init__("assignment1_command_bridge_2027")
        self.last_command = None
        self.envelope = self.create_publisher(
            RoverDriveCommand2027, "/assignment1/validated_drive_command_2027", 1)
        self.subscription = self.create_subscription(
            RoverDriveCommand2027, "/assignment1/rover_drive_command_2027", self.on_command, 1)
        # Using a steady timer so pausing /clock does not suspend the watchdog
        self.watchdog = self.create_timer(
            0.05, self.on_timeout, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def publish(self, velocity, angle):
        """Forward an atomic pair of fixture setpoints; the downstream gate owns controller output."""
        self.envelope.publish(RoverDriveCommand2027(
            front_left_velocity=velocity, front_left_steering=angle))

    def on_command(self, msg):
        """Validate all six fields; reject unsupported actuators instead of ignoring them."""
        values = (msg.front_left_velocity, msg.rear_left_velocity, msg.front_right_velocity,
                  msg.rear_right_velocity, msg.front_left_steering, msg.front_right_steering)
        unsupported = (msg.rear_left_velocity, msg.front_right_velocity,
                       msg.rear_right_velocity, msg.front_right_steering)
        if (not all(math.isfinite(value) for value in values)
                or any(value != 0.0 for value in unsupported)
                or abs(msg.front_left_velocity) > 6.0
                or abs(msg.front_left_steering) > 0.6):
            self.last_command = None
            self.publish(0.0, 0.0)
            self.get_logger().warning("Rejected unsupported or invalid test-stand command; stopping")
            return
        self.last_command = time.monotonic()
        self.publish(msg.front_left_velocity, msg.front_left_steering)

    def on_timeout(self):
        """Stop and center after 0.5 seconds without a valid received command."""
        if self.last_command is None or time.monotonic() - self.last_command > 0.5:
            self.publish(0.0, 0.0)


def main():
    """Run the simulation-only bridge and request zero on normal shutdown."""
    rclpy.init()
    bridge = CommandBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            bridge.publish(0.0, 0.0)
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
