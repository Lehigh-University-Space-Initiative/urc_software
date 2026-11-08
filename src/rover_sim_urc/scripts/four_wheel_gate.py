#!/usr/bin/env python3
"""Sole four-wheel controller publisher; independent monotonic 0.5 s lease.

Repeated stale valid envelopes renew the receipt-time lease: no replay protection.
The separate health monitor contains gate hangs by shutting down the simulation.
Competing DDS publishers and physical braking are not addressed.
"""
import time
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from cross_pkg_messages.msg import RoverDriveCommand2027
from std_msgs.msg import Float64MultiArray, UInt64
from four_wheel_bridge import PREFIX, validated, run

class FourWheelGate(Node):
    def __init__(self):
        super().__init__("four_wheel_gate")
        self.last = None
        self.values = (0.0,) * 4
        self.progress = self.create_publisher(UInt64, PREFIX + "/gate_progress", 1)
        self.output = self.create_publisher(Float64MultiArray, PREFIX + "/wheel_velocity_controller/commands", 1)
        self.input = self.create_subscription(RoverDriveCommand2027, PREFIX + "/validated_command", self.receive, 1)
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, msg):
        values = validated(msg)
        self.last = time.monotonic() if values is not None else None
        self.values = values if values is not None else (0.0,) * 4
        self.tick()

    def tick(self):
        if self.last is None or time.monotonic() - self.last >= 0.5:
            self.last = None
            self.values = (0.0,) * 4
        self.output.publish(Float64MultiArray(data=list(self.values)))
        # Same-host monotonic time: progress after publish returns, not delivery acknowledgement.
        self.progress.publish(UInt64(data=time.monotonic_ns()))

if __name__ == "__main__":
    run(FourWheelGate)
