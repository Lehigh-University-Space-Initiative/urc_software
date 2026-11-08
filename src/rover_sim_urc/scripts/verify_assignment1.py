#!/usr/bin/env python3
"""Command the Assignment 1 simulation and fail unless measured joint motion follows.

Run on the development PC after assignment1.launch.py (or from the launch test).
Publishes Float64MultiArray commands to /assignment1/{wheel_velocity,steering_position}_controller/commands.
Subscribes to /assignment1/joint_state_broadcaster/joint_states (sensor_msgs/JointState).
This exercises simulated actuators directly, before integrating the 2027 rover command node.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class MotionProbe(Node):
    """Keep the latest valid measured sample and publish independent actuator commands."""

    def __init__(self, direct_commands=True):
        super().__init__("assignment1_motion_probe")
        self.sample = None
        self.sequence = 0
        # Typed-message probes must not introduce even unused direct-command publishers
        self.wheel = self.create_publisher(
            Float64MultiArray, "/assignment1/wheel_velocity_controller/commands", 10) if direct_commands else None
        self.steering = self.create_publisher(
            Float64MultiArray, "/assignment1/steering_position_controller/commands", 10) if direct_commands else None
        self.subscription = self.create_subscription(
            JointState, "/assignment1/joint_state_broadcaster/joint_states",
            self.on_state, qos_profile_sensor_data)

    def on_state(self, msg):
        """Accept only finite feedback for both named joints, including wheel velocity."""
        try:
            wheel = msg.name.index("front_left_wheel_joint")
            steering = msg.name.index("front_left_steering_joint")
            sample = (msg.position[wheel], msg.velocity[wheel], msg.position[steering])
        except (ValueError, IndexError):
            return
        if all(math.isfinite(value) for value in sample):
            self.sample = sample
            self.sequence += 1

    def command(self, velocity, angle):
        """Publish wheel rad/s and steering radians to this simulation's controllers."""
        self.wheel.publish(Float64MultiArray(data=[velocity]))
        self.steering.publish(Float64MultiArray(data=[angle]))

    def phase(self, velocity, angle, timeout=20.0):
        """Require fresh feedback, sustained setpoint tracking, and actual wheel travel.

        Args:
            velocity: Wheel angular velocity in rad/s
            angle: Front steering position in radians
            timeout: Wall-clock deadline, so a stopped simulation fails
        """
        deadline = time.monotonic() + timeout
        sequence = self.sequence
        previous = None
        travel = 0.0
        matched_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            self.command(velocity, angle)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.sequence == sequence:
                continue
            sequence = self.sequence
            position, measured_velocity, measured_angle = self.sample
            if previous is not None:
                # Accommodating continuous joints that wrap their reported angle
                travel += math.atan2(math.sin(position - previous), math.cos(position - previous))
            previous = position
            matches = abs(measured_velocity - velocity) < 0.15 and abs(measured_angle - angle) < 0.04
            if matches:
                if matched_since is None:
                    matched_since = time.monotonic()
                    travel = 0.0
                duration = time.monotonic() - matched_since
                displacement_ok = (travel * math.copysign(1.0, velocity) > 0.3
                                   if velocity else abs(travel) < 0.04)
                if duration >= 0.5 and displacement_ok:
                    print(f"PASS: command=({velocity:.2f} rad/s, {angle:.2f} rad), "
                          f"measured=({measured_velocity:.3f} rad/s, {measured_angle:.3f} rad), "
                          f"wheel travel={travel:.3f} rad", flush=True)
                    return
            else:
                matched_since = None
                travel = 0.0
        raise RuntimeError(f"Motion verification timed out: command=({velocity}, {angle}), "
                           f"last feedback={self.sample}, samples={self.sequence}")


def main():
    """Verify rest, positive motion, reverse motion, and stop; always send zero on exit."""
    rclpy.init()
    probe = MotionProbe()
    try:
        probe.phase(0.0, 0.0, timeout=60.0)
        probe.phase(2.0, 0.3)
        probe.phase(-1.5, -0.25)
        probe.phase(0.0, 0.0)
        print("PASS: Assignment 1 ROS 2/Gazebo wheel and front steering motion verified", flush=True)
    finally:
        probe.command(0.0, 0.0)
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
