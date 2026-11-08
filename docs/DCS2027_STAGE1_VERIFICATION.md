# DCS 2027 Stage 1 — software/mock verification, 2026-11-08

## Scope and stopping point

Implemented a separate opt-in `drivetrain_2027_urc` ROS package. It does not
replace legacy deployed nodes, connect to the suspended simulation, open CAN,
configure motors, or publish controller commands. Physical motor output capability
is absent. Stage 1 is not a completed physical migration or a safety-rated controller.

The existing provisional C++ kinematics header already had the correct half-track
formula and remains untouched. A small standard-library Python core implements
that same formula for the standalone mock path, avoiding the main-computer arm,
video and hardware dependency tree. Radius=0.1080 m, track=0.5013 m. Wheelbase is
not needed by the nominal skid formula; no Ackermann/rocker geometry is assumed.

## Changed files

Modified, retaining earlier work:

- `src/cross_pkg_messages_urc/CMakeLists.txt`: add the new service to interface generation.
- `docs/SOFTWARE_PROGRESS.md`: append this milestone and incomplete regression status.

Added:

- `src/cross_pkg_messages_urc/srv/SetDriveMode2027.srv`
- `src/drivetrain_2027_urc/CMakeLists.txt`
- `src/drivetrain_2027_urc/package.xml`
- `src/drivetrain_2027_urc/README.md`
- `src/drivetrain_2027_urc/launch/stage1_mock.launch.py`
- `src/drivetrain_2027_urc/scripts/dcs_core.py`
- `src/drivetrain_2027_urc/scripts/dcs_stage1.py`
- `src/drivetrain_2027_urc/test/test_core.py`
- `src/drivetrain_2027_urc/test/test_stage1_launch.py`
- `docs/DCS2027_STAGE1_VERIFICATION.md`

The original RoverDriveCommand2027 definition, provisional C++ header, all legacy
and arm code, shared CANDriver/MotorManager, all rover_sim_urc implementation,
models/controllers/launch/shutdown logic and its ten tests remain unchanged.
Baseline file hashes are at `/tmp/lusi-stage1-baseline.json`.

## Interface and behavior

All endpoints are isolated under `/dcs_stage1`; full details are in the package README.
Manual input uses the unchanged RoverDriveCommand2027. Autonomy uses TwistStamped
with linear.x in m/s and angular.z in rad/s. The mode service has DISABLED, MANUAL
and AUTONOMY constants. JointState feedback is explicitly named `mock_feedback`.
The only application publishers are `mock_command` and JSON `status`, always
reporting physical_outputs_enabled=false and feedback_is_mock=true.

Startup is DISABLED. Every mode change discards the old command and stops wheel
requests. Enabling requires fresh stopped feedback; an outstanding nonzero wheel
request or moving feedback rejects a transition and leaves DISABLED. AUTONOMY
requires both measured mock front steering positions within 0.02 rad of zero.
It never treats a requested zero angle as proof of centering. Loss of centering
while autonomous latches a fault.

Selected commands and feedback must be finite and fresh. Positive system-time
stamps must be normalized, nonfuture, strictly increasing per source, and below
0.5 s age. Commands must postdate mode selection. Only empty (explicit implicit
body/actuator coordinates) or base_link frame IDs are allowed; no transform occurs.
Autonomous non-x/z axes are rejected. Delayed input receives only its remaining
age budget. Leases use monotonic time and a 50 ms steady timer. Output repetition,
inactive-source input, and repeated same-mode calls cannot renew a command lease.

Invalid selected input, timeout, feedback failure, centering loss or emergency
assertion latches DISABLED. Fault output requests zero wheels and holds the last
bounded mock steering measurement; it does not automatically center on fault.
Deasserting emergency input alone does not reset. Explicit reset requires released
emergency input and fresh stopped feedback and leaves the mode DISABLED. A separate
mode request and subsequent fresh command are required.

The ±100 degree steering bound is the supplied software limit. Speed cap 6 rad/s,
wheel-stop tolerance 0.05 rad/s, center tolerance 0.02 rad, and 0.5 s deadlines are
mock-test settings, not approved physical motor or stopping limits.

## Actual tests and all observed failures

Pinned image:
`sha256:f2b95edaa669f69a6a785dbff0187cfeb61ad950d77962b58b2dd5dead62c603`.
Humble/Gazebo Classic 11.10.2, non-root user, read-only source/root and explicit
writable runtime mounts. Fresh builds of messages, Stage 1 and rover_sim_urc.
No packages or images were installed/built. Existing GUI container was untouched.

Host deterministic tests initially ran 31 cases with two failures: the test helper
asserted the mode before calling poll after advancing its fake clock. Both asserted
against an unexecuted control tick. Preserved source/output:
`/tmp/lusi-dcs-stage1/initial-unit-source/` and `host-core-tests.txt`.
The helper was corrected to execute poll before asserting expired state; no
production timeout or safety assertion was relaxed. Host rerun: 31/31 passed,
recorded in `host-core-tests-r2.txt`.

Both container runs passed all new tests:

| Check | Run 1 | Final-source run 2 |
|---|---:|---:|
| Deterministic core cases | 31/31 | 31/31 |
| Real ROS interface scenario | Passed | Passed |
| Stage 1 CTest duration | 6.66 s | 6.31 s |
| Stage 1 colcon result entries | 34, zero errors/failures/skips | 34, zero errors/failures/skips |
| Command publication cessation to observed mock disabled zero | 0.515 s | 0.516 s |

The final source also rejects wrong frames and non-normalized stamp representations.
The ROS test explicitly covered these added checks. Runtime code and tests for
run 1 are preserved separately under `source-first-run/`; final hashes/snapshot
are under the second run directory.

Actual observed ROS mock requests:

- Independent manual wheels: `(1.0, -1.0, 2.0, -2.0)` rad/s.
- v=0.108 m/s, yaw=0: `(1.0, 1.0, 1.0, 1.0)` rad/s.
- v=0, yaw=+1 rad/s: `(-2.3208333333333333, -2.3208333333333333,
  2.3208333333333333, 2.3208333333333333)` rad/s.
- Command loss, invalid/stale selected command, uncentered autonomy feedback,
  feedback loss and emergency assertion all produced DISABLED zero-wheel requests.
- Centering requests alone did not enable autonomy; new centered feedback was required.
- Inactive manual requests did not override autonomous requests.
- Emergency release alone did not rearm; explicit reset left DISABLED.
- Publisher graph contained only mock/status plus ROS infrastructure publishers.

These are synthetic policy/ROS outputs, not measured physical motion or a mock
plant dynamics simulation. The new policy was not connected to Gazebo controllers.

The unchanged ten-test Gazebo suite ran serially twice:

| Existing test | Run 1 | Run 2 |
|---|---|---|
| test_assignment1_launch.py | Passed 9.39 s | Passed 6.66 s |
| test_command_bridge_2027_launch.py | Passed 18.05 s | Passed 18.23 s |
| test_command_bridge_failure_launch.py | Passed 6.78 s | Passed 6.46 s |
| test_command_gate_launch.py | Passed 24.47 s | Passed 23.66 s |
| test_command_gate_startup_launch.py | Passed 6.33 s | Passed 6.60 s |
| test_four_wheel_launch.py | Passed 25.81 s | Passed 26.01 s |
| test_gate_health_monitor_launch.py | FAILED 5.99 s | Passed 5.96 s |
| test_gate_health_failure_launch.py | Passed 8.08 s | Passed 10.65 s |
| test_ros_cleanup_launch.py | Passed 1.35 s | Passed 1.59 s |
| test_four_wheel_shutdown_launch.py | Passed 22.45 s | FAILED 13.90 s |

Run 1 Gazebo CTest total=128.71 s; run 2=119.74 s. Each run finished 9/10,
so **there is no fully passing ten-test regression run for this milestone**.
Combined new/existing colcon totals in each run: 54 entries, 0 errors,
2 failures, 0 skipped (the one underlying Gazebo failure is also counted by its wrapper).

Run 1 failed because the standalone monitor test never discovered its stale-input
subscriber: `AssertionError: unexpectedly None : stale test never discovered monitor`.
The stale monitor expired without receiving test input. This is failed test coverage,
not proof that repeated stale progress renewed its deadline. Root cause was not fixed.

Run 2 failed before the moving SIGINT scenario: ListControllers returned only
`{'joint_state_broadcaster': 'active'}` where both controllers were required.
The wheel-controller spawner was still waiting for its service when test cleanup
began. This establishes a startup synchronization failure, not a confirmed cause
in Stage 1 code. No test readiness check or deadline was changed to make it pass.
Idle shutdown in that run passed in 0.470 s; the moving/monitor subscenarios were
not completed by that test. The separate gate-SIGSTOP regression passed with
0.531 s detection, 0.988 s clean Gazebo exit and 1.848 s full launch exit.

Historical Gazebo self-join and other shutdown concerns remain unresolved; this
milestone did not change or diagnose them. Existing headless graphics/audio,
Classic lifecycle and controller update/interface warnings remain.

## Evidence and exact stopping point

- `/tmp/lusi-dcs-stage1/`: host failures/fix output, first source snapshot, first
  complete container run including its failed regression.
- `/tmp/lusi-dcs-stage1-r2/`: final source snapshot, second full container run.
- Each run has `run.sh`, `source-before.json`, `output/console.txt`, test-result
  summaries, build/test logs, Stage 1 XML and all ten Gazebo XML reports.
- SHA256SUMS and source-preservation checks are recorded alongside artifacts.
- Prior preserved bundles and `/tmp/lusi-assignment1-ros-cleanup` were not modified.

Stage 1 implementation and its new tests are complete, but overall regression
validation is incomplete. Execution testing stopped after the second run rather
than retrying until green or weakening preserved safety checks. No physical
backend, deployed-node cutover, motor enable, steering controller, joystick/GPS
migration, replay authentication, DDS authorization, or ground locomotion was added.

Before any physical work: independently validate firmware/protocol, ID/polarity
mapping, encoder calibration and freshness, exact steering ratio, limits, fault
hold/disable behavior, hardwired emergency response, and a downstream watchdog.
Timestamp checks assume synchronized system clocks and cannot authenticate fresh
forgeries. A hung arbiter, false mock feedback and monitor/scheduler faults remain
outside these assurances. Software leases and zero requests do not prove braking.

Next review should preserve this evidence and resolve or isolate the unchanged
regression startup/discovery failures without weakening their assertions. Physical
Stage 2 remains unimplemented and requires a separate approval.
