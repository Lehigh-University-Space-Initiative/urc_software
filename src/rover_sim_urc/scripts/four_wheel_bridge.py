#!/usr/bin/env python3
"""Simulation-only four-wheel message validation; no kinematics or hardware output.

Wheel order: front left, rear left, front right, rear right. Both steering
fields must be zero because the source SDF has no steering joints.
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from cross_pkg_messages.msg import RoverDriveCommand2027

PREFIX = "/assignment1_four_wheel"
FIELDS = ("front_left_velocity", "rear_left_velocity", "front_right_velocity", "rear_right_velocity")

def validated(msg):
    """Return finite, bounded simulation setpoints or reject the entire envelope."""
    values = tuple(getattr(msg, field) for field in FIELDS)
    steering = (msg.front_left_steering, msg.front_right_steering)
    if (not all(math.isfinite(v) for v in values + steering)
            or any(v != 0.0 for v in steering) or any(abs(v) > 6.0 for v in values)):
        return None
    return values

class FourWheelBridge(Node):
    """Own upstream validation and receipt-time timeout; gate owns controller output."""
    def __init__(self):
        super().__init__("four_wheel_bridge")
        self.last = None
        self.output = self.create_publisher(RoverDriveCommand2027, PREFIX + "/validated_command", 1)
        self.input = self.create_subscription(RoverDriveCommand2027, PREFIX + "/rover_drive_command_2027", self.receive, 1)
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def publish(self, values):
        self.output.publish(RoverDriveCommand2027(**dict(zip(FIELDS, values))))

    def receive(self, msg):
        values = validated(msg)
        self.last = time.monotonic() if values is not None else None
        self.publish(values if values is not None else (0.0,) * 4)

    def tick(self):
        if self.last is None or time.monotonic() - self.last >= 0.5:
            self.publish((0.0,) * 4)

def cleanup(node):
    """Destroy ROS entities without publishing; clean the default context owned by run()."""
    try:
        if node is not None:
            node.destroy_node()
    finally:
        rclpy.try_shutdown()

def run(cls):
    """Run bridge or gate; shutdown containment does not rely on a final publication."""
    rclpy.init()
    node = None
    try:
        node = cls()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cleanup(node)

if __name__ == "__main__":
    run(FourWheelBridge)
