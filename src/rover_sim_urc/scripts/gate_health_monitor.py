#!/usr/bin/env python3
"""Same-host simulation gate progress watchdog; exit triggers supervised shutdown.

No actuator publishers. This contains missing gate progress, not stale drive
commands, spoofed advancing heartbeats, controller failure, or physical motion.
"""
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import UInt64


class GateHealthMonitor(Node):
    """Accept only fresh increasing monotonic timestamps; fail closed on timeout."""

    def __init__(self):
        super().__init__("gate_health_monitor")
        self.started = time.monotonic_ns()
        self.last_progress = None
        self.expired = False
        self.subscription = self.create_subscription(
            UInt64, "/assignment1_four_wheel/gate_progress", self.receive, 1)
        self.timer = self.create_timer(
            0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, msg):
        now = time.monotonic_ns()
        # An expired lease is terminal, even if a fresh callback races the next timer.
        self.tick()
        if self.expired:
            return
        if (0 < msg.data <= now and now - msg.data < 500_000_000
                and (self.last_progress is None or msg.data > self.last_progress)):
            self.last_progress = msg.data

    def tick(self):
        now = time.monotonic_ns()
        deadline = (self.started + 3_000_000_000 if self.last_progress is None
                    else self.last_progress + 500_000_000)
        if not self.expired and now >= deadline:
            self.expired = True
            self.get_logger().error("Gate progress deadline expired; requesting simulation shutdown")


def main():
    rclpy.init()
    monitor = GateHealthMonitor()
    try:
        while rclpy.ok() and not monitor.expired:
            rclpy.spin_once(monitor, timeout_sec=0.05)
        return 2 if monitor.expired else 0
    except KeyboardInterrupt:
        return 0
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
