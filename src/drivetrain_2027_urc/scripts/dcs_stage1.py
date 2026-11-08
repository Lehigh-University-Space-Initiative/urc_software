#!/usr/bin/env python3
"""Software-only ROS adapter. Publishes mock commands, never controller/CAN output.

Manual input: RoverDriveCommand2027. Autonomous input: standard TwistStamped.
All stamped inputs use synchronized system time; watchdogs use monotonic time.
Only the explicit mock feedback topic is consumed. No physical enable exists.
"""
import json

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from cross_pkg_messages.msg import RoverDriveCommand2027
from cross_pkg_messages.srv import SetDriveMode2027
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from dcs_core import Arbiter, Command, finite

PREFIX = "/dcs_stage1"
FIELDS = ("front_left_velocity", "rear_left_velocity", "front_right_velocity", "rear_right_velocity")
JOINTS = tuple(f.replace("_velocity", "_wheel_joint") for f in FIELDS) + (
    "front_left_steering_joint", "front_right_steering_joint")


def stamp_ns(header):
    if (header.frame_id not in ("", "base_link") or header.stamp.sec < 0
            or not 0 <= header.stamp.nanosec < 1_000_000_000):
        return 0
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class Stage1(Node):
    def __init__(self):
        super().__init__("dcs_stage1_mock_arbiter")
        self.policy = Arbiter()
        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.output = self.create_publisher(RoverDriveCommand2027, PREFIX + "/mock_command", 1)
        self.state = self.create_publisher(String, PREFIX + "/status", 1)
        self.manual_sub = self.create_subscription(RoverDriveCommand2027, PREFIX + "/manual_command", self.manual, 1)
        self.auto_sub = self.create_subscription(TwistStamped, PREFIX + "/autonomy_cmd_vel", self.autonomy, 1)
        self.feedback_sub = self.create_subscription(JointState, PREFIX + "/mock_feedback", self.feedback, 1)
        self.emergency_sub = self.create_subscription(Bool, PREFIX + "/emergency_stop", self.emergency, 1)
        self.mode_service = self.create_service(SetDriveMode2027, PREFIX + "/set_mode", self.set_mode)
        self.reset_service = self.create_service(Trigger, PREFIX + "/reset_fault", self.reset)
        self.timer = self.create_timer(0.05, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def publish(self):
        command = self.policy.poll()
        msg = RoverDriveCommand2027(**dict(zip(FIELDS, command.wheels)),
                                   front_left_steering=command.steering[0],
                                   front_right_steering=command.steering[1])
        msg.header.stamp = self.system_clock.now().to_msg()
        self.output.publish(msg)
        self.state.publish(String(data=json.dumps(self.policy.status(), sort_keys=True)))

    def manual(self, msg):
        self.policy.manual(Command(tuple(getattr(msg, f) for f in FIELDS),
                                   (msg.front_left_steering, msg.front_right_steering)), stamp_ns(msg.header))
        self.publish()

    def autonomy(self, msg):
        self.policy.autonomy((msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z),
                             (msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z), stamp_ns(msg.header))
        self.publish()

    def feedback(self, msg):
        if (len(msg.name) != 6 or len(set(msg.name)) != 6 or set(msg.name) != set(JOINTS)
                or len(msg.position) != 6 or len(msg.velocity) != 6 or len(msg.effort) not in (0, 6)):
            self.policy.feedback_deadline = None
            self.policy.trip("MALFORMED_MOCK_FEEDBACK")
        else:
            indices = [msg.name.index(j) for j in JOINTS]
            # Validate the complete feedback frame, including otherwise unused components.
            if not finite((*msg.position, *msg.velocity, *msg.effort)):
                self.policy.feedback_deadline = None
                self.policy.trip("NONFINITE_MOCK_FEEDBACK")
            else:
                self.policy.feedback(tuple(msg.velocity[i] for i in indices[:4]),
                                     tuple(msg.position[i] for i in indices[4:]), stamp_ns(msg.header))
        self.publish()

    def emergency(self, msg):
        self.policy.emergency_stop(msg.data)
        self.publish()

    def set_mode(self, request, response):
        response.accepted, response.reason = self.policy.request_mode(request.mode)
        response.current_mode = int(self.policy.mode)
        self.publish()
        return response

    def reset(self, request, response):
        response.success, response.message = self.policy.reset_fault()
        self.publish()
        return response


def main():
    rclpy.init()
    node = None
    try:
        node = Stage1()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
