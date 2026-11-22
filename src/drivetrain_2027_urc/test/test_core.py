"""Deterministic mock-hardware policy tests; no ROS or physical transport."""
from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from dcs_core import Arbiter, Command, Mode, STEERING_LIMIT, skid_steer


class Clock:
    def __init__(self):
        self.mono = 0.0
        self.ns = 1_000_000_000_000

    def advance(self, seconds):
        self.mono += seconds
        self.ns += round(seconds * 1e9)

    def stamp(self):
        self.advance(0.001)
        return self.ns


class TestKinematics(unittest.TestCase):
    def test_straight_and_reverse(self):
        self.assertEqual(skid_steer(0.108, 0).wheels, (1.0,) * 4)
        self.assertEqual(skid_steer(-0.108, 0).wheels, (-1.0,) * 4)

    def test_yaw_sign_and_half_track(self):
        command = skid_steer(0, 1.0)
        for value in command.wheels[:2]:
            self.assertAlmostEqual(value, -2.3208333333333333)
        for value in command.wheels[2:]:
            self.assertAlmostEqual(value, 2.3208333333333333)
        self.assertEqual(command.steering, (0.0, 0.0))

    def test_combined_motion(self):
        c = skid_steer(0.108, 0.2)
        self.assertAlmostEqual(c.wheels[0], (0.108 - 0.2 * 0.5013 / 2) / 0.108)
        self.assertAlmostEqual(c.wheels[2], (0.108 + 0.2 * 0.5013 / 2) / 0.108)

    def test_nonfinite(self):
        for v in [float("nan"), float("inf"), -float("inf")]:
            with self.assertRaises(ValueError):
                skid_steer(v, 0)


class TestPolicy(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.policy = Arbiter(lambda: self.clock.mono, lambda: self.clock.ns)

    def feedback(self, wheels=(0.0,) * 4, steering=(0.0, 0.0)):
        return self.policy.feedback(wheels, steering, self.clock.stamp())

    def arm(self, mode=Mode.MANUAL):
        self.feedback()
        self.assertTrue(self.policy.request_mode(mode)[0])

    def manual(self, wheels=(1.0,) * 4, steering=(0.0, 0.0), stamp=None):
        return self.policy.manual(Command(wheels, steering), self.clock.stamp() if stamp is None else stamp)

    def advance_feedback(self, duration):
        steps = math.ceil(duration / 0.1)
        for _ in range(steps):
            self.clock.advance(duration / steps)
            self.feedback()

    def assert_disarmed(self, reason=None):
        self.policy.poll()  # Advance the control tick after the injected clock change.
        self.assertEqual(self.policy.mode, Mode.DISABLED)
        self.assertEqual(self.policy.poll().wheels, (0.0,) * 4)
        self.assertFalse(self.policy.status()["physical_outputs_enabled"])
        if reason is not None:
            self.assertEqual(self.policy.fault, reason)

    def test_startup_disabled_no_feedback(self):
        self.assert_disarmed()
        self.assertFalse(self.manual())
        self.assertFalse(self.policy.request_mode(Mode.MANUAL)[0])
        self.assert_disarmed()

    def test_manual_independent_fields(self):
        self.arm()
        self.assertTrue(self.manual((1, -1, 2, -2), (0.3, -0.4)))
        self.assertEqual(self.policy.poll(), Command((1, -1, 2, -2), (0.3, -0.4)))

    def test_steering_exact_limits(self):
        self.arm()
        self.assertTrue(self.manual(steering=(STEERING_LIMIT, -STEERING_LIMIT)))

    def test_steering_outside_limit(self):
        self.arm()
        self.assertFalse(self.manual(steering=(STEERING_LIMIT + 1e-6, 0)))
        self.assert_disarmed("INVALID_OR_STALE_MANUAL")

    def test_nonfinite_manual_values(self):
        self.arm()
        self.assertFalse(self.manual((1, float("nan"), 1, 1)))
        self.assert_disarmed("INVALID_OR_STALE_MANUAL")

    def test_manual_speed_limit(self):
        self.arm()
        self.assertFalse(self.manual((6.0001, 0, 0, 0)))
        self.assert_disarmed("INVALID_OR_STALE_MANUAL")

    def test_command_loss_latches(self):
        self.arm()
        self.manual()
        self.advance_feedback(0.51)
        self.assert_disarmed("COMMAND_TIMEOUT")
        self.assertFalse(self.manual())

    def test_enable_without_command_expires(self):
        self.arm()
        self.advance_feedback(0.51)
        self.assert_disarmed("COMMAND_TIMEOUT")

    def test_feedback_loss(self):
        self.arm()
        self.manual()
        self.clock.advance(0.501)
        self.assert_disarmed("FEEDBACK_TIMEOUT")

    def test_late_feedback_cannot_hide_expiration(self):
        self.arm()
        self.manual()
        self.clock.advance(0.501)
        self.feedback()
        self.assert_disarmed("FEEDBACK_TIMEOUT")

    def test_inactive_source_cannot_renew_lease(self):
        self.arm()
        self.manual()
        for _ in range(6):
            self.clock.advance(0.1)
            self.feedback()
            self.policy.autonomy((0.108, 0, 0), (0, 0, 0), self.clock.stamp())
        self.assert_disarmed("COMMAND_TIMEOUT")

    def test_output_poll_cannot_renew_lease(self):
        self.arm()
        self.manual()
        for _ in range(6):
            self.clock.advance(0.1)
            self.feedback()
            self.policy.poll()
        self.assert_disarmed("COMMAND_TIMEOUT")

    def test_zero_future_and_old_headers(self):
        self.arm()
        self.assertFalse(self.manual(stamp=0))
        self.assert_disarmed("INVALID_OR_STALE_MANUAL")
        for stamp in [self.clock.ns + 1, self.clock.ns - 600_000_000]:
            self.feedback()
            self.policy.reset_fault()
            self.policy.request_mode(Mode.MANUAL)
            self.assertFalse(self.manual(stamp=stamp))
            self.assert_disarmed("INVALID_OR_STALE_MANUAL")

    def test_duplicate_stamp_rejected(self):
        self.arm()
        stamp = self.clock.stamp()
        self.assertTrue(self.manual(stamp=stamp))
        self.clock.advance(0.1)
        self.assertFalse(self.manual(stamp=stamp))
        self.assert_disarmed("INVALID_OR_STALE_MANUAL")

    def test_delayed_command_only_gets_remaining_budget(self):
        self.arm()
        old = self.clock.stamp()
        self.clock.advance(0.4)
        self.feedback()
        self.assertTrue(self.manual(stamp=old))
        self.clock.advance(0.11)
        self.feedback()
        self.assert_disarmed("COMMAND_TIMEOUT")

    def test_pretransition_command_rejected(self):
        self.feedback()
        stamp = self.clock.ns
        self.policy.request_mode(Mode.MANUAL)
        self.assertFalse(self.manual(stamp=stamp))
        self.assert_disarmed("INVALID_OR_STALE_MANUAL")

    def test_same_mode_does_not_renew_lease(self):
        self.arm()
        self.manual()
        for _ in range(6):
            self.clock.advance(0.1)
            self.feedback()
            self.policy.request_mode(Mode.MANUAL)
        self.assert_disarmed("COMMAND_TIMEOUT")

    def test_transition_stops_and_discards_command(self):
        self.arm()
        self.manual()
        self.assertFalse(self.policy.request_mode(Mode.AUTONOMY)[0])
        self.assert_disarmed()
        self.assertTrue(self.policy.request_mode(Mode.AUTONOMY)[0])
        self.assertEqual(self.policy.poll().wheels, (0.0,) * 4)

    def test_measured_motion_blocks_enable(self):
        self.feedback(wheels=(0.1, 0, 0, 0))
        self.assertFalse(self.policy.request_mode(Mode.MANUAL)[0])
        self.assert_disarmed()

    def test_autonomy_requires_center_feedback(self):
        self.feedback(steering=(0.3, -0.2))
        self.assertFalse(self.policy.request_mode(Mode.AUTONOMY)[0])
        self.assert_disarmed()
        self.assertEqual(self.policy.poll().steering, (0.3, -0.2))
        self.feedback()
        self.assertTrue(self.policy.request_mode(Mode.AUTONOMY)[0])

    def test_autonomy_output_and_wrong_axes(self):
        self.arm(Mode.AUTONOMY)
        self.assertTrue(self.policy.autonomy((0.108, 0, 0), (0, 0, 0), self.clock.stamp()))
        self.assertEqual(self.policy.poll(), Command((1.0,) * 4))
        self.assertFalse(self.policy.autonomy((0, 0, 0), (0, 1, 0), self.clock.stamp()))
        self.assert_disarmed("INVALID_AUTONOMY_AXES")

    def test_autonomy_speed_overflow_rejected(self):
        self.arm(Mode.AUTONOMY)
        self.assertFalse(self.policy.autonomy((1e308, 0, 0), (0, 0, 0), self.clock.stamp()))
        self.assert_disarmed("INVALID_OR_STALE_AUTONOMY")

    def test_center_loss_during_autonomy_faults(self):
        self.arm(Mode.AUTONOMY)
        self.feedback(steering=(0.021, 0))
        self.assert_disarmed("STEERING_NOT_CENTERED")
        self.assertEqual(self.policy.poll().steering, (0.021, 0))

    def test_invalid_feedback_requires_new_valid_sample(self):
        self.arm()
        self.feedback(steering=(2, 0))
        self.assert_disarmed("INVALID_OR_STALE_FEEDBACK")
        self.assertFalse(self.policy.reset_fault()[0])
        self.feedback()
        self.assertTrue(self.policy.reset_fault()[0])
        self.assert_disarmed()

    def test_emergency_deassertion_does_not_rearm(self):
        self.arm()
        self.manual()
        self.policy.emergency_stop(True)
        self.assert_disarmed("EMERGENCY_STOP")
        self.assertFalse(self.policy.reset_fault()[0])
        self.policy.emergency_stop(False)
        self.assert_disarmed("EMERGENCY_STOP")
        self.feedback()
        self.assertTrue(self.policy.reset_fault()[0])
        self.assert_disarmed()
        self.assertTrue(self.policy.request_mode(Mode.MANUAL)[0])
        self.assertEqual(self.policy.poll().wheels, (0.0,) * 4)

    def test_invalid_mode(self):
        self.arm()
        self.assertFalse(self.policy.request_mode(99)[0])
        self.assert_disarmed("INVALID_MODE")

    def test_wall_clock_jump_cannot_extend_existing_lease(self):
        self.arm()
        self.manual()
        deadline = self.policy.command_deadline
        self.clock.ns -= 10_000_000_000
        self.assertEqual(self.policy.command_deadline, deadline)
        self.clock.mono += 0.6
        self.assert_disarmed("FEEDBACK_TIMEOUT")


if __name__ == "__main__":
    unittest.main(verbosity=2)
