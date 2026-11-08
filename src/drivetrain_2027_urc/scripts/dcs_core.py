"""Stage 1 DCS policy only: no ROS, CAN, motor API, or physical output capability.

The speed/tolerance/deadline values below are mock-test policy, not approved
physical limits. Header checks assume synchronized system clocks; they do not
provide publisher authentication or protection against forged fresh messages.
"""
from dataclasses import dataclass
from enum import IntEnum
import math
import time


class Mode(IntEnum):
    DISABLED = 0
    MANUAL = 1
    AUTONOMY = 2


@dataclass(frozen=True)
class Command:
    wheels: tuple = (0.0, 0.0, 0.0, 0.0)  # FL, RL, FR, RR; wheel rad/s
    steering: tuple = (0.0, 0.0)  # FL, FR; output-joint rad


WHEEL_RADIUS = 0.1080
TRACK_WIDTH = 0.5013
STEERING_LIMIT = math.radians(100.0)
MAX_MOCK_WHEEL_SPEED = 6.0
LEASE = 0.5
CENTER_TOLERANCE = 0.02
STOP_TOLERANCE = 0.05


def finite(values):
    try:
        return all(math.isfinite(v) for v in values)
    except (TypeError, ValueError):
        return False


def skid_steer(v, yaw_rate):
    """Nominal planar skid kinematics; positive yaw is ROS counterclockwise."""
    if not finite((v, yaw_rate)):
        raise ValueError("Nonfinite body command")
    left = (v - yaw_rate * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
    right = (v + yaw_rate * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
    return Command((left, left, right, right))


class Arbiter:
    """Single-threaded mock policy with explicit mode, input leases, and latched faults."""

    physical_outputs_enabled = False  # Informational invariant, never a hardware enable.

    def __init__(self, monotonic=time.monotonic, system_ns=time.time_ns):
        self.monotonic = monotonic
        self.system_ns = system_ns
        self.mode = Mode.DISABLED
        self.fault = ""
        self.emergency_active = False
        self.last_stamp = {"manual": 0, "autonomy": 0, "feedback": 0}
        self.mode_epoch = 0
        self.command_deadline = None
        self.feedback_deadline = None
        self.wheel_feedback = (0.0,) * 4
        self.steering_feedback = (0.0,) * 2
        self.command = Command()

    def _disarm(self):
        self.mode = Mode.DISABLED
        self.command_deadline = None
        # Stop drive requests, hold the last bounded mock steering measurement.
        # Never automatically center steering in response to a fault.
        self.command = Command(steering=self.steering_feedback)

    def trip(self, reason):
        if not self.fault:
            self.fault = reason
        self._disarm()

    def emergency_stop(self, active):
        self.emergency_active = bool(active)
        if active:
            self.trip("EMERGENCY_STOP")
        # Deassertion alone does not clear the latch or rearm a mode.

    def _stamp_deadline(self, stamp, source):
        now_ns = self.system_ns()
        if (not isinstance(stamp, int) or stamp <= self.last_stamp[source]
                or stamp <= 0 or stamp > now_ns):
            return None
        age = (now_ns - stamp) / 1e9
        if age >= LEASE:
            return None
        # A delayed input gets only its remaining age budget, not a fresh full lease.
        return self.monotonic() + LEASE - age

    def feedback_fresh(self):
        return self.feedback_deadline is not None and self.monotonic() < self.feedback_deadline

    def stopped(self):
        return all(abs(v) <= STOP_TOLERANCE for v in self.wheel_feedback)

    def centered(self):
        return all(abs(v) <= CENTER_TOLERANCE for v in self.steering_feedback)

    def feedback(self, wheels, steering, stamp):
        self.poll()  # A late sample must not erase an already expired feedback lease.
        deadline = self._stamp_deadline(stamp, "feedback")
        if (len(wheels) != 4 or len(steering) != 2 or not finite((*wheels, *steering))
                or any(abs(v) > MAX_MOCK_WHEEL_SPEED for v in wheels)
                or any(abs(v) > STEERING_LIMIT for v in steering) or deadline is None):
            self.feedback_deadline = None
            self.trip("INVALID_OR_STALE_FEEDBACK")
            return False
        self.last_stamp["feedback"] = stamp
        self.feedback_deadline = deadline
        self.wheel_feedback = tuple(wheels)
        self.steering_feedback = tuple(steering)
        if self.mode == Mode.DISABLED:
            self.command = Command(steering=self.steering_feedback)
        self.poll()
        return True

    def request_mode(self, requested):
        self.poll()
        try:
            requested = Mode(requested)
        except (ValueError, TypeError):
            self.trip("INVALID_MODE")
            return False, "Unknown mode"
        if requested == Mode.DISABLED:
            self._disarm()
            return True, "Disabled; faults remain latched"
        if self.fault or self.emergency_active:
            self._disarm()
            return False, "Reset required after fault/emergency"
        if requested == self.mode:
            return True, "Mode unchanged; command lease not renewed"
        was_commanding_motion = any(v != 0.0 for v in self.command.wheels)
        self._disarm()  # No old command survives a transition attempt.
        if not self.feedback_fresh():
            return False, "Fresh mock feedback required"
        if was_commanding_motion or not self.stopped():
            return False, "Stop and verify wheels before changing mode"
        if requested == Mode.AUTONOMY and not self.centered():
            return False, "Center both front joints and verify feedback first"
        self.mode = requested
        self.mode_epoch = self.system_ns()
        self.command_deadline = self.monotonic() + LEASE
        return True, "Mode selected; waiting for a new source command"

    def reset_fault(self):
        self.poll()
        if self.mode != Mode.DISABLED or self.emergency_active:
            return False, "Disable and release emergency input first"
        if not self.feedback_fresh() or not self.stopped():
            return False, "Fresh stopped mock feedback required"
        self.fault = ""
        return True, "Fault cleared; mode remains DISABLED"

    def _accept(self, command, stamp, source, mode):
        self.poll()
        if self.mode != mode:
            return False  # Inactive sources cannot renew or replace the selected source.
        deadline = self._stamp_deadline(stamp, source)
        if (deadline is None or stamp <= self.mode_epoch
                or len(command.wheels) != 4 or len(command.steering) != 2
                or not finite((*command.wheels, *command.steering))
                or any(abs(v) > MAX_MOCK_WHEEL_SPEED for v in command.wheels)
                or any(abs(v) > STEERING_LIMIT for v in command.steering)):
            self.trip("INVALID_OR_STALE_" + source.upper())
            return False
        self.last_stamp[source] = stamp
        self.command_deadline = deadline
        self.command = command
        return True

    def manual(self, command, stamp):
        return self._accept(command, stamp, "manual", Mode.MANUAL)

    def autonomy(self, linear, angular, stamp):
        self.poll()
        # Twist axes are x-forward / z-yaw only. Reject the legacy angular.y convention.
        if (len(linear) != 3 or len(angular) != 3 or not finite((*linear, *angular))
                or any(v != 0.0 for v in (linear[1], linear[2], angular[0], angular[1]))):
            if self.mode == Mode.AUTONOMY:
                self.trip("INVALID_AUTONOMY_AXES")
            return False
        return self._accept(skid_steer(linear[0], angular[2]), stamp, "autonomy", Mode.AUTONOMY)

    def poll(self):
        if self.mode != Mode.DISABLED:
            if not self.feedback_fresh():
                self.trip("FEEDBACK_TIMEOUT")
            elif self.mode == Mode.AUTONOMY and not self.centered():
                self.trip("STEERING_NOT_CENTERED")
            elif self.command_deadline is None or self.monotonic() >= self.command_deadline:
                self.trip("COMMAND_TIMEOUT")
        return self.command

    def status(self):
        self.poll()
        return {"mode": self.mode.name, "fault": self.fault,
                "emergency_active": self.emergency_active,
                "physical_outputs_enabled": False, "feedback_is_mock": True,
                "feedback_fresh": self.feedback_fresh()}
