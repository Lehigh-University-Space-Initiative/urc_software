#!/usr/bin/env python3
"""Independently expire simulated actuator commands downstream of the 2027 bridge.

Started by assignment1_command_2027.launch.py as a separate development-PC process.
Subscribes to /assignment1/validated_drive_command_2027 (RoverDriveCommand2027).
Sole controller-command publisher in that launch: Float64MultiArray wheel rad/s and steering rad.
Receipt-time freshness is not replay protection; repeated stale envelopes can renew the lease.
All limits and zero steering targets belong to the existing fixture, not physical hardware.
"""

import math
import time

from cross_pkg_messages.msg import RoverDriveCommand2027
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class CommandGate(Node):
    """Own controller output and enforce a 0.5 s lease independently of bridge execution."""

    def __init__(self):
        super().__init__("assignment1_command_gate_2027")
        self.last_valid = None
        self.velocity = 0.0
        self.angle = 0.0
        self.wheel = self.create_publisher(
            Float64MultiArray, "/assignment1/wheel_velocity_controller/commands", 1)
        self.steering = self.create_publisher(
            Float64MultiArray, "/assignment1/steering_position_controller/commands", 1)
        self.subscription = self.create_subscription(
            RoverDriveCommand2027, "/assignment1/validated_drive_command_2027", self.on_command, 1)
        self.timer = self.create_timer(
            0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_command(self, msg):
        """Independently validate every field; rejected envelopes stop without renewing the lease."""
        values = (msg.front_left_velocity, msg.rear_left_velocity, msg.front_right_velocity,
                  msg.rear_right_velocity, msg.front_left_steering, msg.front_right_steering)
        unsupported = (msg.rear_left_velocity, msg.front_right_velocity,
                       msg.rear_right_velocity, msg.front_right_steering)
        if (not all(math.isfinite(value) for value in values)
                or any(value != 0.0 for value in unsupported)
                or abs(msg.front_left_velocity) > 6.0
                or abs(msg.front_left_steering) > 0.6):
            self.last_valid = None
            self.velocity = self.angle = 0.0
        else:
            self.last_valid = time.monotonic()
            self.velocity = msg.front_left_velocity
            self.angle = msg.front_left_steering
        self.tick()

    def tick(self):
        """Expire using monotonic time and repeatedly publish zero until a fresh valid envelope."""
        if self.last_valid is None or time.monotonic() - self.last_valid >= 0.5:
            self.last_valid = None
            self.velocity = self.angle = 0.0
        self.wheel.publish(Float64MultiArray(data=[self.velocity]))
        self.steering.publish(Float64MultiArray(data=[self.angle]))


def main():
    """Run the gate; process-exit supervision contains abrupt failures in the isolated launch."""
    rclpy.init()
    gate = CommandGate()
    try:
        rclpy.spin(gate)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            gate.last_valid = None
            gate.tick()
        gate.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
