# LUSI URC 2027 — Software Progress

Last reviewed: 2026-11-08

## Scope and status definitions

This document records verified results and unresolved dependencies.
Source presence alone does not establish working behavior.

- Verified: demonstrated by identified test evidence in the stated environment
- Partial: a bounded part of a subsystem is verified
- Unknown: current behavior has not been assessed with sufficient evidence
- Blocked: a stated dependency prevents the next correct integration step

Append dated milestones without deleting earlier history.
Record verification scope, commands, environment, evidence, and limitations.
Do not infer whole-rover or hardware readiness from simulation results.

## Repository baseline

- Branch: feature/2027-drivetrain-migration
- Starting HEAD: 1686e21f82d6cfd9994370b4d7876684569053fd
- Assignment 1 development remains uncommitted
- Eight preexisting changed/untracked files match initial SHA-256 hashes
- Original four-wheel and six-wheel simulation assets are preserved
- Physical hardware control code and provisional kinematics are unchanged
- Five submodules and the original stash remain preserved

Source preservation is verified; unrelated runtime functionality is not.
The baseline review did not rerun the full rover build or hardware tests.

## Subsystem status

| Subsystem | Current status | Source locations and verification boundary |
|---|---|---|
| Assignment 1 drivetrain simulation | Verified, limited to two-joint fixture | src/rover_sim_urc; five Gazebo integration tests passed |
| Complete 2027 drivetrain | Partial; full-rover steering blocked | Existing command message is generated and used by the fixture; four-wheel steering and complete actuator mapping are unimplemented |
| Physical driveline/CAN | Unknown | src/driveline_urc; not changed or hardware-tested by Assignment 1 |
| Main-computer coordination | Unknown | src/main_computer_urc; existing runtime orchestration not retested |
| Ground station, GUI, operator input | Unknown | src/base_station_urc; no verified end-to-end 2027 operator command path |
| Arm hardware/control | Unknown | src/arm_urc; no new verification |
| MoveIt and arm controller configuration | Unknown | src/moveit_config_urc and src/main_computer_urc; no new verification |
| Navigation/autonomy | Unknown | src/navigation_urc; no integration with the new simulation command path verified |
| Science mechanisms and sensors | Unknown | src/science_urc; existing source/history does not establish current hardware readiness |
| Video and camera transport | Unknown | src/main_computer_urc and src/base_station_urc; not retested |
| LiDAR and perception | Unknown | src/slidar_ros2; ROS package name sllidar_ros2; perception pipeline readiness not assessed |
| Shared ROS interfaces | Partial | src/cross_pkg_messages_urc; RoverDriveCommand2027 generation and fixture use verified; other interfaces not retested |
| Shared hardware libraries | Unknown | src/shared_code and libs; preserved but not newly runtime-verified |
| State-machine/voice integration | Unknown | No local voice-command package identified; external work was not refreshed during this review |
| Build, deployment, networking, CI | Partial | Isolated Humble simulation build verified; full rover build, deployment, network faults, and CI remain unverified |
| Backup and recovery | Verified historical accessible-data recovery | Phase 0 backup/recovery passed with ownership and cache limitations; not repeated during this review |

## Verified command path

```text
/assignment1/rover_drive_command_2027
  -> scripts/command_bridge_2027.py
  -> /assignment1/validated_drive_command_2027
  -> scripts/command_gate_2027.py
  -> wheel velocity and steering position forward controllers
  -> Gazebo joints
  -> measured JointState feedback
```

Only front_left_velocity and front_left_steering are supported.
The four unsupported actuator fields must be zero.

The bridge validates upstream requests and retains its upstream timeout.
The independent gate validates received envelopes and owns controller output.

Gate lease:

- 0.5 seconds since a valid received envelope
- Monotonic time and a steady timer checked every 50 ms
- Invalid envelopes clear the lease and request zero
- Expiration repeatedly requests zero wheel velocity and zero fixture steering
- Fresh valid input restores motion
- Repeated controller output does not renew the input lease

Bridge or gate process exit requests launch-wide simulation shutdown.
This configuration must remain isolated from unrelated launch sessions.

## Assignment 1 implementation paths

All following paths are relative to src/rover_sim_urc:

- CMakeLists.txt
- package.xml
- Dockerfile
- README.md
- config/assignment1_controllers.yaml
- description/assignment1.urdf.xacro
- worlds/assignment1.world
- launch/assignment1.launch.py
- launch/assignment1_command_2027.launch.py
- scripts/command_bridge_2027.py
- scripts/command_gate_2027.py
- scripts/verify_assignment1.py
- scripts/verify_command_bridge_2027.py

Detailed historical evidence:
docs/ASSIGNMENT1_VERIFICATION.md

Preexisting dependencies:

- src/cross_pkg_messages_urc/msg/RoverDriveCommand2027.msg
- src/cross_pkg_messages_urc/CMakeLists.txt
- src/rover_sim_urc/models/lusi_rover/model.sdf
- src/rover_sim_urc/models/lusi_rover/model.config
- src/rover_sim_urc/models/lusi_rover_6wheel_backup/model.sdf
- src/rover_sim_urc/models/lusi_rover_6wheel_backup/model.config

Related preexisting local-only work, unchanged and not used by the fixture:

- src/main_computer_urc/src/DriveTrainManager2027/DriveKinematics2027.hpp
- docs/gazebo

## Five verified integration tests

Test paths are relative to src/rover_sim_urc/test:

| Test | Verified scope |
|---|---|
| test_assignment1_launch.py | Model spawn, direct controller commands, positive/reverse wheel motion, steering targets, stopping |
| test_command_bridge_2027_launch.py | Typed command path, invalid upstream requests, silence timeout, recovery |
| test_command_bridge_failure_launch.py | Scoped bridge SIGKILL during measured motion; clean simulation shutdown |
| test_command_gate_launch.py | Sole output publisher, bridge SIGSTOP, independent expiration, sustained zero output, ten invalid gate inputs, recovery, gate SIGKILL |
| test_command_gate_startup_launch.py | Startup without a bridge or envelope publisher; repeated zero output and stopped feedback |

Final recorded environment:

- ROS 2 Humble
- Gazebo Classic 11.10.2
- Approved lusi-assignment1-sim image
- Non-root test user; source directories mounted read-only
- ROS domain 85
- Gazebo master http://127.0.0.1:11385
- ROVER_SIM_TEST_2027_COMMAND_BRIDGE=ON
- Tests executed serially

Final recorded results:

- Five integration tests passed
- Zero failures or errors
- CTest duration: 67.62 seconds
- Colcon reports ten result entries because it also counts CTest wrappers

| Measurement | Result |
|---|---:|
| Bridge SIGSTOP to measured stopping/centering | 0.542 s |
| Gate-input silence to measured stopping/centering | 0.558 s |
| Bridge SIGKILL to clean simulation shutdown | 0.377 s |
| Gate SIGKILL to clean simulation shutdown | 0.508 s |
| Startup without envelope publishers | 100 stopped samples over 1.0 s; 20 zero outputs per controller topic |

Measured motion in the verified fixture included:

- Wheel 2.000 rad/s with steering 0.300 rad
- Wheel -1.500 rad/s with steering -0.250 rad
- Signed wheel displacement and sustained stopping checks

Evidence:

- /tmp/lusi-assignment1-command-gate-r2/run.sh
- /tmp/lusi-assignment1-command-gate-r2/console.txt
- /tmp/lusi-assignment1-command-gate-r2/test-result.txt
- /tmp/lusi-assignment1-command-gate-r2/build/rover_sim_urc/test_results/rover_sim_urc/
- /tmp/lusi-assignment1-command-gate-r2/ros-logs/

These /tmp artifacts are temporary and are not portable repository evidence.
Preserve durable test artifacts before publishing the baseline.

## Dated milestones

All entries below occurred on 2026-11-08; ordering distinguishes milestones.

1. Phase 0: accessible-data backup and independent recovery passed.
   Root-owned Mesa caches were intentionally omitted as reproducible data.
   Original ownership metadata was preserved; recovery did not restore all
   root ownership. No byte-for-byte cache recovery is claimed.

2. Initial fixture: Gazebo loaded a suspended front-left wheel/steering
   assembly. Direct ROS commands produced measured forward/reverse motion,
   steering, and stopping.

3. Typed command bridge: RoverDriveCommand2027 controlled the fixture.
   Unsupported, nonfinite, and out-of-range requests were rejected.
   Receipt-time command silence produced measured stopping.

4. Process failure containment: bridge SIGKILL caused simulation shutdown.
   Earlier launch-test shutdown and output-stream harness failures were
   corrected and retained in docs/ASSIGNMENT1_VERIFICATION.md.

5. Independent command gate: bridge SIGSTOP no longer prevented expiration.
   The gate became the sole configured controller-command publisher.
   Startup, rejection, expiration, sustained stopping, recovery, and gate
   exit passed. Both gate milestone suite runs had no build/test failures.

6. Baseline review: branch, tracked diff, untracked implementation, manifests,
   five result XML files, and preservation boundaries reviewed read-only.
   No fresh build or simulation was executed during this review.

## Unresolved risks and test gaps

- Replay protection is absent. Repeated old or delayed valid envelopes can
  renew the receipt-time lease. Header timestamps and ordering are unchecked.
- A hung gate remains a failure mode; exit supervision detects exit, not hangs.
- Supervisor, controller-manager, Gazebo, and DDS failures are not covered.
- Competing controller publishers are not blocked by DDS permissions.
- Two controller output topics are not a transactional actuator update.
  Partial delivery and asymmetric communication failures are untested.
- Operational stopping does not depend on joint-feedback health; feedback
  disappearance has not been tested as a stop trigger.
- Paused-world, reset/clock-transition, restart, overload, and long-duration
  behavior remain unverified.
- The startup test checks zero output and feedback but does not explicitly
  assert that both command controllers are active first.
- Controller activation failures and controller deactivation/restart faults
  are not directly exercised.
- Invalid-input coverage is representative, not an exhaustive field/value
  matrix or a complete exact-limit boundary test.
- Normal shutdown attempts a final zero publish without delivery confirmation.
- Process-exit containment terminates the simulator; it does not prove
  physical braking or measured zero velocity after process death.
- GUI rendering, contact dynamics, whole-rover motion, and physical hardware
  remain unverified.
- The standalone Docker recipe runs only the original test by default.
  The full suite requires the message overlay and opt-in CMake flag.
- Docker reproduction depends on a local mutable builder image and package
  repository state. Fresh-checkout reproduction and full rover CI are unverified.
- Package license metadata remains TODO.
- Confluence access previously failed; conflicting internal work has not
  been fully ruled out. Remote updates were not refreshed in this review.

## Mechanical and Electrical dependencies

Existing four-wheel SDF:

- Four drive joints directly attached to the chassis; no steering joints
- Wheel radius: 0.1016 m
- Wheel-center track: 0.80 m
- Wheelbase: 0.72 m

Preexisting provisional kinematics:

- Wheel radius: 0.108 m
- Track: 0.5013 m
- Skid-steer helper centers steering; no steering-linkage implementation

Neither set is accepted as confirmed physical geometry.

Full-rover steering requires confirmed:

- Steering-axis origins, orientations, and linkage topology
- Steering travel and interference limits
- Wheel dimensions, track, wheelbase, offsets, masses, and inertias
- Motor/encoder mappings, gearing, signs, and calibration for hardware work

The fixture's ±6 rad/s and ±0.6 rad limits are simulation parameters only.
No hardware-control change should derive specifications from this fixture.

## Next recommended integration step

First establish a reproducible repository baseline:
approve the prerequisite/implementation commit boundaries, preserve durable
evidence, and reproduce all five tests from a clean checkout in isolation.

Then prioritize a controlled gate-SIGSTOP investigation and an independent
response to gate hangs before extending safety claims or adding more actuators.

Full-rover steering remains gated on confirmed mechanical specifications.
Other subsystems remain Unknown until their own evidence is recorded.

## 2026-11-08 — Reproduced baseline and four-wheel drive-joint milestone

This addition supersedes the earlier next-step and two-joint-only status where
explicitly stated; prior results and paths above remain historical evidence.

Clean-checkout reproduction v3 passed all five tests in 52.92 s, zero errors,
failures or skips. Preserved evidence:
`/home/rek429/urc-reproduction/assignment1-20261108-r1/output/`.
Recorded HEAD, five submodule revisions and 185 source hashes matched.
This resolves the earlier open question of reproduction with the pinned local
image and packaged uncommitted overlay; fresh image reconstruction remains open.

After approval, added a separate suspended four-wheel fixture derived from
`src/rover_sim_urc/models/lusi_rover/model.sdf`. Original assets and two-joint
configuration remain unchanged. The four original drive-joint names, frames,
axes, geometry sizes, masses and inertias were checked against the derived
configuration. No steering joints or physical kinematics were added.

New implementation paths:

- `src/rover_sim_urc/launch/four_wheel.launch.py`
- `src/rover_sim_urc/config/four_wheel_controllers.yaml`
- `src/rover_sim_urc/scripts/four_wheel_bridge.py`
- `src/rover_sim_urc/scripts/four_wheel_gate.py`
- `src/rover_sim_urc/test/test_four_wheel_launch.py`

Package registration changes:
`src/rover_sim_urc/CMakeLists.txt` and `src/rover_sim_urc/package.xml`.
Detailed commands, exact results, warnings, preservation checks and limitations
are appended in `docs/ASSIGNMENT1_VERIFICATION.md`.

The four velocity fields in RoverDriveCommand2027 directly command the matching
joints under `/assignment1_four_wheel`; both steering fields must be zero.
Independent validation, monotonic 0.5 s expiration checked every 50 ms,
repeated zero output, recovery and process-exit supervision are included.
No new physical specifications are inferred from this model.

Six Gazebo tests ran serially in fresh directories using the pinned approved
image and read-only source/root. All five original tests passed unchanged;
the new four-wheel test passed. CTest: 77.14 s. Colcon: 12 result entries,
**zero errors, failures and skips**. No build/test failures occurred.
The explicit controller-service test dependency was declared after the run;
its package was already installed and the manifest was XML-validated.

Measured evidence:

| Check | Result |
|---|---|
| Each wheel individually, both signs | ±1.500 rad/s; three inactive wheels remained zero |
| All four coordinated forward | 2.000 rad/s each; 1.020 rad displacement each |
| All four coordinated reverse | -1.500 rad/s each; -0.750 rad displacement each |
| Explicit stop and sustained stop | All four zero within test tolerance; fresh feedback and negligible travel |
| Upstream silence | Stop 0.482 s after test publishing ceased |
| Bridge SIGSTOP | Independent stop 0.503 s; 100 stopped samples over following second |
| Gate-input silence | Independent stop 0.550 s; subsequent sustained stop |
| Invalid direct gate envelopes | Six cases stopped before expiration; 0.320–0.329 s includes 0.3 s stable observation |
| Recovery | Fresh upstream command restored -1.500 rad/s on all four |
| Gate SIGKILL | Clean simulation shutdown in 0.340 s |

Evidence root: `/tmp/lusi-assignment1-four-wheel/`; console, six XML reports,
launch logs, ROS/Gazebo logs and build logs remain under `output/`.
The evidence is temporary until separately preserved durably.

Current subsystem scope: drivetrain simulation is now verified for independent
rotation of four suspended drive joints as well as the original two-joint
wheel/steering stand. Complete 2027 drivetrain remains Partial. Physical CAN,
operator input, autonomous navigation, arm/MoveIt, science, video, LiDAR,
shared hardware libraries and state-machine integration retain their earlier
Unknown or Partial statuses; no new subsystem integration was tested.

Mechanical dependencies remain confirmed steering topology, frames, travel,
geometry and interference limits. Electrical dependencies remain motor and
encoder IDs, gearing, polarity, calibration and hardware safety behavior.
These do not block this bounded suspended joint test, but do block physical
integration and steering claims.

Unresolved risks carry forward: replay, hung gate, supervisor/controller/DDS
faults, competing publishers, feedback-loss response, pause/reset transitions,
and physical stopping. Four-wheel bridge-exit supervision was not independently
fault-injected; its two-joint counterpart remains covered. No chassis
locomotion, traction, odometry or steering is claimed.

Next recommended step: preserve this six-test evidence and review the isolated
four-wheel mapping, then investigate gate SIGSTOP and an independent response
to a hung gate before extending safety claims. Steering/hardware integration
requires confirmed Mechanical and Electrical specifications.

## 2026-11-08 — Independent four-wheel gate-health monitor

This entry updates the preceding gate-hang limitation for the tested isolated
simulation. Earlier milestones, measurements, evidence paths and other subsystem
statuses remain historical and unchanged.

Added a separate same-host monitor that watches monotonic timestamp progress
published after the four-wheel gate's controller output callback. It accepts
only fresh strictly increasing values. Equal/stale/future values cannot renew
its deadline. Startup without progress expires after three seconds; subsequent
progress expires after 0.5 seconds, checked every 50 ms using a steady clock.
Monitor exit triggers launch-wide simulation shutdown. No additional controller
publisher was added. Stopped-gate signal escalation is bounded in launch.

Exact new paths:

- `src/rover_sim_urc/scripts/gate_health_monitor.py`
- `src/rover_sim_urc/test/test_gate_health_monitor_launch.py`
- `src/rover_sim_urc/test/test_gate_health_failure_launch.py`

Existing paths changed only for heartbeat/monitor registration and supervision:
`src/rover_sim_urc/scripts/four_wheel_gate.py`,
`src/rover_sim_urc/launch/four_wheel.launch.py`, and
`src/rover_sim_urc/CMakeLists.txt`.
Detailed verification is appended in `docs/ASSIGNMENT1_VERIFICATION.md`.
All six existing tests, models, bridge, message, hardware and kinematics were preserved.

The final eight-test Gazebo/ROS suite passed serially in fresh directories using
the pinned image, read-only source/root, domain 89 and Gazebo port 11389.
CTest: **89.59 s**. Colcon: **16 entries, 0 errors, 0 failures, 0 skipped**,
including eight wrappers. No dependencies or images were installed/rebuilt.

| Measured result | Value |
|---|---:|
| Four wheel velocities before gate SIGSTOP | 2.000 rad/s each |
| SIGSTOP to detected deadline | 0.552 s |
| SIGSTOP to clean Gazebo shutdown | 0.990 s |
| SIGSTOP to full launch shutdown | 1.794 s |
| Healthy advancing progress without a launched /clock source | Sustained 4 s |
| Silent startup to exit observation from test-loop start | 3.557 s, includes startup overhead |
| Repeated constant heartbeat: first send to process exit | 0.696 s |
| Fresh progress cessation to process exit | 0.695 s |

This is shutdown containment, not measured wheel braking. Callback progress is
not confirmation of controller receipt or hardware response. Startup grace is
bounded but is not an actuator-enable interlock.

Failures are retained rather than erased:

- `/tmp/lusi-assignment1-gate-health/`: initial suite had two failed tests.
  One unchanged regression encountered a Gazebo shutdown self-join abort (-6).
  One new harness test incorrectly read returncode from ProcessStarted; corrected
  only that new test. Colcon counted 1 error and 3 failures including wrappers.
- `/tmp/lusi-assignment1-gate-health-r2/`: final suite passed. It nevertheless
  logged an unchanged bridge final-publish/ROS-context shutdown race (exit 1)
  during the gate-SIGSTOP test. Bounded clean Gazebo shutdown still passed.
  This runtime exception and the earlier Gazebo abort are not fixed or dismissed.

Both directories retain logs, XML, commands and source hashes/snapshots.
Durable preservation remains pending. The previously verified durable four-wheel
six-test bundle was checksum-checked and not changed.

Remaining risks: stale drive-command replay, forged advancing heartbeats,
competing controller publishers, monitor/supervisor hangs, feedback/controller/DDS
faults, and nondeterministic shutdown behavior. There is no hard real-time bound,
remote-host clock support, physical stopping verification, or ground-contact claim.
Other subsystem and Mechanical/Electrical dependencies retain their prior status.

Next recommended step: durably preserve both runs and review shutdown reliability,
including Gazebo self-join and bridge final-publish/context races, before extending
safety claims or undertaking ground-contact locomotion. No commits or pushes.

## 2026-11-08 — Shared ROS shutdown cleanup corrected

The diagnosed four-wheel bridge teardown race is corrected in
`src/rover_sim_urc/scripts/four_wheel_bridge.py`, whose `run()` helper is also used
by `src/rover_sim_urc/scripts/four_wheel_gate.py`. Cleanup no longer publishes a
final command. It handles ROS-initiated executor termination, attempts node
destruction, and always attempts idempotent default-context shutdown, including
constructor/destruction failures. Unexpected live errors still propagate.
Running command validation, 0.5 s expiration, independent gate-health monitoring,
controller publishers, models and launch signal order/escalation are unchanged.

Added `src/rover_sim_urc/test/test_ros_cleanup_launch.py` and
`src/rover_sim_urc/test/test_four_wheel_shutdown_launch.py`, registered in
`src/rover_sim_urc/CMakeLists.txt`. All eight existing tests remain unchanged.
Full detail and exact evidence paths are appended in
`docs/ASSIGNMENT1_VERIFICATION.md`; prior history is retained.

The complete ten-test suite passed serially in fresh output directories using
the pinned image, read-only source/root and domain 90/Gazebo port 11390.
CTest **105.14 s**, colcon **20 entries, 0 errors, 0 failures, 0 skipped** including
wrappers. No build/test failures occurred. Both bridge and gate passed forced
context invalidation, repeated cleanup, constructor/destruction failure handling,
and live-error propagation checks.

Actual Gazebo feedback before moving shutdown was 2.000 rad/s on each wheel.
Normal idle/moving SIGINT full shutdowns were **0.366/0.366 s**; monitor-triggered
shutdown was **1.876 s**. Bridge and Gazebo clean exits were required in every
new scenario; normal scenarios also required clean gate exits. No traceback or
invalid-context publication error appeared in the console or three scenario logs.
The separate unchanged monitor regression detected gate SIGSTOP in **0.543 s**,
with clean Gazebo exit at **0.941 s** and full launch exit at **1.788 s**.
Existing command expiration, stopping, signed motion and recovery checks passed.

Evidence: `/tmp/lusi-assignment1-ros-cleanup/`, including exact build/test commands,
source hashes/snapshot, ten XML reports, logs and three complete shutdown-scenario
logs. Checksummed temporary evidence requires separate durable preservation.
No packages/images were installed/built and no commits or staging occurred.
All eight preexisting file hashes, existing models/tests, hardware code, message,
kinematics, gate, monitor and launch files remain preserved.

The earlier bridge shutdown exception remains part of historical evidence; this
entry records its narrow correction. The historical Gazebo thread self-join
abort did not recur here, but remains unresolved. Tested simulation termination
is not measured wheel braking or physical safety validation. Other subsystem
statuses and Mechanical/Electrical dependencies remain unchanged.

Next recommended step: preserve this evidence, then obtain a native backtrace
for the historical Gazebo abort before changing signal order or extending claims.
No steering, ground-contact, replay-protection or hardware work was added.

## 2026-11-08 — DCS migration Stage 1: isolated software/mock path

Added opt-in `src/drivetrain_2027_urc`, isolated from physical motor and existing
simulation topics. It provides DISABLED/MANUAL/AUTONOMY arbitration, distinct
stamped manual/autonomy inputs, corrected nominal skid kinematics (radius 0.1080 m,
track 0.5013 m), fresh stopped mock-feedback requirements, and measured-mock
front-centering prerequisites for autonomy. The existing provisional C++ helper
and RoverDriveCommand2027 definition remain unchanged.

Commands/feedback use header-age/order checks plus monotonic 0.5 s leases checked
by a 50 ms steady timer. Selected invalid/stale input, lost feedback, lost autonomy
centering and mock emergency input latch DISABLED. Explicit reset does not rearm.
Physical output capability is absent; the node publishes only mock requests and
status. No physical backend, CAN/arm/shared-driver change, deployed cutover or
Gazebo controller/shutdown change occurred.

New files and full interface contract are listed in
`docs/DCS2027_STAGE1_VERIFICATION.md` and
`src/drivetrain_2027_urc/README.md`. The only shared-interface build change is
registration of `src/cross_pkg_messages_urc/srv/SetDriveMode2027.srv`.

Both container runs passed 31 deterministic policy/kinematics cases and one real
ROS mock-interface scenario (34 colcon entries including wrappers). Final-source
ROS results: v=0.108 m/s produced four 1.000 rad/s requests; yaw=+1 rad/s produced
left -2.320833/right +2.320833 rad/s requests; command loss produced observed
DISABLED zero-wheel requests in 0.516 s. These are mock requests, not physical
feedback or new Gazebo integration.

Initial host unit execution had two harness failures from checking mode before
executing the poll tick after fake-clock advancement. The test helper was fixed;
all 31 then passed. No production timeout or assertion was weakened.

**Existing regression validation is incomplete.** The unchanged ten-test Gazebo
suite ran twice, each finishing 9/10. First run: stale-heartbeat test never
discovered its subscriber. Second run: shutdown test queried controllers while
only joint_state_broadcaster was active and the wheel spawner was still waiting.
The previously failed monitor test passed on run 2. No existing test, timeout,
readiness assertion, model or shutdown handler was changed. Combined colcon
results per run: 54 entries, 0 errors, 2 failures, 0 skipped (one failed underlying
regression plus its wrapper). Full pass/fail tables are in the verification document.

Evidence is preserved separately at `/tmp/lusi-dcs-stage1/` and
`/tmp/lusi-dcs-stage1-r2/`, including source snapshots, commands, XML and logs.
These temporary artifacts require a separate durable preservation step. Previous
evidence and all existing uncommitted work remain retained.

Stopping point: mock-only Stage 1 implemented and tested; no physical Stage 2.
Remaining risks include synchronized-clock assumptions, forged fresh input/false
feedback, arbiter hangs, and unverified physical fault behavior. Existing Gazebo
startup/discovery failures and historical self-join abort remain unresolved.
Other subsystem statuses and Mechanical/Electrical dependencies are unchanged.
Next: preserve evidence and review the regression failures before any deployment
or physical backend work. No commits, staging, pushes, reset or cleaning.
