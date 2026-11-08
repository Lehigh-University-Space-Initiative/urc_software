# DCS 2027 Stage 1 — software and mock feedback only

This opt-in package does not replace a deployed driveline node. It has no CAN
backend, physical controller publisher, or hardware-enable option. Never remap
its mock output into a physical actuator consumer. The simulation package and
legacy driveline/arm paths remain independent.

The existing `RoverDriveCommand2027` message and provisional C++ kinematics header
are unchanged. This standalone package avoids coupling Stage 1 builds to the
main-computer arm/video dependencies. It implements the same nominal skid formula
with radius 0.1080 m and track 0.5013 m; it does not implement traction prediction,
Ackermann steering, gearing, encoder calibration, or motor control.

Build `cross_pkg_messages` first, then `drivetrain_2027_urc` with BUILD_TESTING=ON.
Run `ros2 launch drivetrain_2027_urc stage1_mock.launch.py` in an isolated ROS domain.
Run `colcon test --packages-select drivetrain_2027_urc --ctest-args -j1` and inspect
`colcon test-result --verbose`. Existing ten Gazebo regressions remain registered
in rover_sim_urc under its original two opt-in test flags.

## ROS interface

All paths below are under `/dcs_stage1`:

| Endpoint | Type | Meaning |
|---|---|---|
| manual_command | cross_pkg_messages/RoverDriveCommand2027 | Four wheel rad/s, FL/FR steering joint rad |
| autonomy_cmd_vel | geometry_msgs/TwistStamped | Body-frame linear.x m/s and angular.z rad/s only |
| mock_feedback | sensor_msgs/JointState | Explicitly synthetic four-wheel/two-steering state |
| emergency_stop | std_msgs/Bool | Mock software interlock; true latches a fault |
| set_mode | cross_pkg_messages/SetDriveMode2027 | Request DISABLED=0, MANUAL=1 or AUTONOMY=2 |
| reset_fault | std_srvs/Trigger | Explicit reset; never automatically enables a mode |
| mock_command | cross_pkg_messages/RoverDriveCommand2027 | Software request only; not an actuator output |
| status | std_msgs/String | JSON mode/fault/feedback and invariant physical_outputs_enabled=false |

Feedback requires exactly these names, in any order, with six finite positions
and velocities: `front_left_wheel_joint`, `rear_left_wheel_joint`,
`front_right_wheel_joint`, `rear_right_wheel_joint`,
`front_left_steering_joint`, `front_right_steering_joint`.
Effort is optional; when present it must contain six finite values.
No mock plant or fake calibration is started automatically.

## Policy

- Startup is DISABLED and inputs cannot arm it implicitly.
- Every mode change discards the old command and stops wheel requests.
- Selection requires fresh stopped feedback; an outstanding nonzero wheel request
  or measured wheel motion rejects a transition and leaves DISABLED.
- AUTONOMY additionally requires both measured steering angles within 0.02 rad of
  zero. A zero steering request does not substitute for centered feedback.
  Loss of centering while active latches a fault.
- Inactive command sources are ignored. Repeated mode requests/output ticks cannot
  refresh a command lease. No cached command survives a mode change.
- All stamped inputs require positive, normalized system-time stamps, empty or
  `base_link` frame ID, age below 0.5 s, and strictly increasing stamps per source.
  Commands must also be newer than mode selection. No coordinate transform is
  performed. Empty frame ID explicitly means body/actuator coordinates here.
- Command and feedback deadlines use monotonic time, checked every 50 ms.
  Delayed messages receive only their remaining 0.5 s age budget.
- Selected invalid/stale commands, missing feedback/commands, lost autonomy
  centering and asserted emergency input latch DISABLED. Requests stop wheel
  motion and hold the last bounded mock steering measurement; faults do not
  automatically center steering.
- Clearing emergency input does not reset the latch. Reset requires DISABLED,
  a released emergency input, and fresh stopped feedback. A separate mode request
  and subsequent fresh command are then required.
- Unexpected node exceptions are not hidden. Cleanup publishes no final command.

The ±100 degree steering bound is the supplied DCS software limit. The 6 rad/s
wheel cap, 0.05 rad/s stopped tolerance, 0.02 rad centering tolerance and 0.5 s
leases are **mock-test settings**, not approved physical operating limits.

Headers assume synchronized, nonregressing system clocks. This is not source
authentication or protection against forged fresh timestamps. Feedback is not
proof of calibrated hardware. A process hang, false feedback, scheduler failure,
or injected publisher can invalidate these software assurances. The emergency
input is not a physical emergency stop. A future hardware backend needs separate
independent output inhibition and a validated safety/feedback contract.

Stopping point: Stage 1 arbitration and mock verification only. No changes to
CANDriver, MotorManager, SPARK firmware, motor configuration, physical launches,
existing Gazebo controllers, or shutdown supervision are authorized by this package.
