# Assignment 1 drivetrain test evidence — 2026-11-08

Branch: `feature/2027-drivetrain-migration`, starting HEAD `1686e21`.
The isolated front-left assembly passed a real ROS 2 Humble / Gazebo Classic 11.10.2
headless integration test. This establishes wheel rotation and front steering
control, not four-wheel rover locomotion or physical hardware behavior.

## Implementation

Added package metadata/build installation, a suspended URDF test assembly, an
offline world, a launch file, controller parameters, a motion verifier, and a launch
test under `src/rover_sim_urc`. A separate Dockerfile derives from the existing
`urc_software_builder:latest` image and adds the approved Gazebo ROS packages.
It uses a named non-root user with a writable container home and workspace.

The velocity controller drives `front_left_wheel_joint`; the position controller
drives `front_left_steering_joint`. Feedback comes from Gazebo through
`gazebo_ros2_control` and `joint_state_broadcaster`. Controller commands and measured
joint states use the `/assignment1` namespace. See the package README for commands.

## Environment and checks

- Docker Desktop WSL integration was enabled by the user; client/server both 29.8.0.
- Existing Humble builder contains colcon, CMake, GCC 11.4, controllers, xacro,
  robot_state_publisher, and launch-testing support.
- Both existing rover images lacked `gazebo_ros` and `gazebo_ros2_control`.
- The separate image installed those approved packages and dependencies; existing
  Docker configurations and image tags were not modified.
- The package built in the original builder with isolated `/tmp` outputs:
  `Summary: 1 package finished [3.20s]`.
- The final separate image rebuilt successfully:
  `Summary: 1 package finished [2.37s]`.
- Python syntax, XML, YAML-to-model joint/interface consistency, Gazebo SDF world
  validation, and Gazebo URDF conversion passed.
- A SHA-256 comparison confirmed all eight pre-existing modified/untracked files
  unchanged, including the existing models, 2027 interface, and kinematics.

## Measured integration result

The final test ran as the container's non-root user, using ROS domain 78 and
Gazebo master `http://127.0.0.1:11378`. No repository source or existing rover build
caches were mounted into that test container.

Gazebo reported `Successfully spawned entity [assignment1_drivetrain]` and loaded
both joint command interfaces. All three controller spawners activated successfully.

| Wheel command (rad/s) | Steering command (rad) | Measured wheel (rad/s) | Measured steering (rad) | Wheel travel during observation (rad) |
|---:|---:|---:|---:|---:|
| 0.00 | 0.00 | -0.000 | 0.000 | -0.000 |
| 2.00 | 0.30 | 2.000 | 0.300 | 1.000 |
| -1.50 | -0.25 | -1.500 | -0.250 | -0.810 |
| 0.00 | 0.00 | 0.000 | -0.000 | 0.000 |

The verifier requires fresh, finite samples for both named joints, sustained
setpoint tracking, signed wheel displacement during motion, and minimal displacement
after stopping. It uses wall-clock deadlines rather than simulation timeouts.

```text
PASS: Assignment 1 ROS 2/Gazebo wheel and front steering motion verified
1/1 Test #1: test_test_assignment1_launch.py ... Passed 7.32 sec
100% tests passed, 0 tests failed out of 1
Summary: 2 tests, 0 errors, 0 failures, 0 skipped
```

There is one motion integration test; colcon's summary counts both its CTest wrapper
and launch-test result. Gazebo and robot_state_publisher exited cleanly after testing.

Evidence on this machine:

- `/tmp/lusi-assignment1-results/run3/integration-console.txt`
- `/tmp/lusi-assignment1-results/run3/test-result.txt`
- `/tmp/lusi-assignment1-results/run3/test_results/rover_sim_urc/test_test_assignment1_launch.py.xunit.xml`
- `/tmp/lusi-assignment1-results/run3/launch_test/test_test_assignment1_launch.py.txt`
- `/tmp/lusi-assignment1-results/run3/ros-logs/`
- Initial preservation manifest: `/tmp/lusi-assignment1-original-work.json`

The `/tmp` evidence is temporary; this document preserves its measured summary.

## Earlier failures and remaining limits

- Before Docker integration, the host had no ROS 2, compiler, or colcon; native
  colcon invocation failed. Initial sandbox Gazebo/Docker checks also required
  approved access outside the sandbox.
- The first isolated image build was intentionally interrupted to inspect existing
  environments after the user's instruction. No successful test is claimed for it.
- The first container test invocation discovered zero packages; its log is
  `/tmp/lusi-assignment1-results/integration-console.txt`. Source discovery was corrected.
- The next test failed because a numeric UID without a home made Gazebo try to
  create `/.gazebo`; the feedback timeout correctly failed the motion test. Its log
  is `/tmp/lusi-assignment1-results/integration-console-r2.txt`. The named image user
  corrected that environment issue before the passing run.
- The passing headless run logged unavailable display/audio warnings and Gazebo's
  Classic end-of-life notice. GUI rendering and audio were not tested.
- Controllers use ideal simulated interfaces; motor dynamics, contact forces,
  whole-rover navigation, the 2027 high-level command bridge, and CAN are untested.
  Commands can persist until replaced or the simulator stops.
- Existing four-wheel and six-wheel configurations remain unchanged and available;
  their full runtime behavior and the full rover build were not retested.

## Concurrent work review

GitHub main has newer [state-machine planning (#34)](https://github.com/Lehigh-University-Space-Initiative/urc_software/pull/34)
and [science serial timing (#35)](https://github.com/Lehigh-University-Space-Initiative/urc_software/pull/35)
changes. The open [voice-command PR (#33)](https://github.com/Lehigh-University-Space-Initiative/urc_software/pull/33)
adds only files under `src/voice_commands`. No obvious overlap with this isolated
simulation package was identified; no remote changes were merged or fetched into Git.

The `search-company-knowledge` skill was used to attempt the Confluence check at
`lehigh-lusi.atlassian.net`; search returned `403 Access denied`. Confluence updates
remain unchecked. No commits, pushes, merges, resets, or cleaning were performed.

## Next milestone: 2027 message integration — 2026-11-08

This section records a later passing run. Earlier limitations above describe the
first milestone; the two front-left message fields are now tested through the
simulation-only bridge. The full four-wheel command path remains unimplemented.

### Reviewed model and interfaces

Source: `src/rover_sim_urc/models/lusi_rover/model.sdf`.

| Existing joint | Parent → child | Axis | Wheel center in model coordinates (m) |
|---|---|---|---|
| `front_left_wheel_joint` | `chassis` → `front_left_wheel` | +Y | `(0.36, 0.40, -0.20)` |
| `front_right_wheel_joint` | `chassis` → `front_right_wheel` | +Y | `(0.36, -0.40, -0.20)` |
| `rear_left_wheel_joint` | `chassis` → `rear_left_wheel` | +Y | `(-0.36, 0.40, -0.20)` |
| `rear_right_wheel_joint` | `chassis` → `rear_right_wheel` | +Y | `(-0.36, -0.40, -0.20)` |

All four are revolute joints with wide rotation bounds. The model has **no steering
joints, knuckles, ROS plugins, or controller configuration**. Each wheel is attached
directly to the chassis. Its wheel radius is 0.1016 m and width 0.10 m; the wheel
centers imply a 0.80 m simulated track and 0.72 m wheelbase. The chassis box is
1.0 × 0.65 × 0.18 m; model origin height is 0.32 m. These are existing simulation
values, not confirmed hardware measurements.

`src/cross_pkg_messages_urc/msg/RoverDriveCommand2027.msg` defines four independent
wheel angular velocities (rad/s) and two front steering position targets (rad).
It describes requested motion, not feedback or hardware-output authorization.

`src/main_computer_urc/src/DriveTrainManager2027/DriveKinematics2027.hpp` explicitly
labels its radius 0.108 m and track 0.5013 m as provisional. Its skid-steer helper
uses standard linear.x/angular.z inputs, copies each side's speed to front/rear,
and leaves both steering targets zero. It contains no front steering linkage or
Ackermann implementation. Neither those dimensions nor the SDF values are sufficient
to establish mechanically correct 2027 steering geometry. This milestone does not
call or modify that helper.

### Small integration step and exact changed paths

The existing front-left test stand already has a tested wheel velocity interface,
steering position interface, and position/velocity feedback. Reusing these requires
no new geometry or kinematic calculation. The stand's knuckle dimensions, ±0.6 rad
steering range, and ±6 rad/s wheel range remain fixture parameters only.

Added:

- `src/rover_sim_urc/scripts/command_bridge_2027.py`
- `src/rover_sim_urc/scripts/verify_command_bridge_2027.py`
- `src/rover_sim_urc/launch/assignment1_command_2027.launch.py`
- `src/rover_sim_urc/test/test_command_bridge_2027_launch.py`

Updated only package installation/dependency/test registration and documentation:

- `src/rover_sim_urc/CMakeLists.txt`
- `src/rover_sim_urc/package.xml`
- `src/rover_sim_urc/README.md`
- `docs/ASSIGNMENT1_VERIFICATION.md`

The bridge subscribes to `/assignment1/rover_drive_command_2027`, maps
`front_left_velocity` and `front_left_steering` directly to the existing controller
topics, and requires all other actuator fields to be zero. Unsupported, nonfinite,
or fixture-out-of-range commands stop and center the stand. A 0.5 s receipt-time
watchdog uses a steady clock. This is an isolated simulation adapter; it does not
subscribe to hardware topics or introduce whole-rover kinematics.

The bridge launch includes the original launch without altering it. Its integration
test is opt-in with `ROVER_SIM_TEST_2027_COMMAND_BRIDGE=ON`, because the original
standalone image does not build the generated 2027 message package. Both tests ran
serially in the new overlay to avoid simultaneous access to the same Gazebo master.

### Build and numerical evidence

Reused the existing approved image
`lusi-assignment1-sim@sha256:f2b95edaa669f69a6a785dbff0187cfeb61ad950d77962b58b2dd5dead62c603`.
No image rebuild, package installation, sudo, or host permission change was needed.
Both package source directories were read-only mounts. All new generated files,
builds, installs, and logs are under `/tmp/lusi-assignment1-command-bridge`.

Actual build output:

```text
Finished <<< cross_pkg_messages [13.9s]
Summary: 1 package finished [14.4s]
Finished <<< rover_sim_urc [1.60s]
Summary: 1 package finished [2.03s]
```

Gazebo 11.10.2 spawned the test assembly and loaded the same tested controllers.
ROS domain 79 and Gazebo master `http://127.0.0.1:11379` isolated this run.

| Typed message command | Measured wheel (rad/s) | Measured steering (rad) | Wheel travel (rad) |
|---|---:|---:|---:|
| Front-left `2.0 rad/s`, `0.30 rad` | 2.000 | 0.300 | 1.000 |
| Front-left `-1.5 rad/s`, `-0.25 rad` | -1.500 | -0.250 | -0.765 |
| After command silence | 0.000 | -0.000 | 0.000 during sustained rest |

Measured stop/centering after command silence: **0.560 s**. Rejected inputs produced
measured stop/centering in the following times, all before the 0.5 s watchdog:

| Invalid field / value | Measured stop latency (s) |
|---|---:|
| `rear_left_velocity=1.0` | 0.019 |
| `front_right_velocity=1.0` | 0.020 |
| `rear_right_velocity=1.0` | 0.020 |
| `front_right_steering=0.1` | 0.020 |
| `front_left_velocity=NaN` | 0.020 |
| `front_left_steering=Inf` | 0.020 |
| `front_left_velocity=6.1` | 0.020 |
| `front_left_steering=0.61` | 0.021 |

The verifier uses the original motion probe's measured-feedback checks, overriding
its command method to publish only the typed message. Its inherited direct-controller
publishers are never used. It also recovered valid reverse motion after rejection
and verified a final explicit stop.

```text
100% tests passed, 0 tests failed out of 2
Total Test time (real) = 25.12 sec
Summary: 4 tests, 0 errors, 0 failures, 0 skipped
```

There are two actual integration tests; colcon counts their CTest wrappers and
launch-test reports separately. No build or test failures occurred in this milestone.
The original test passed again. All eight original uncommitted files were verified
unchanged by SHA-256; the original test model, controllers, launch, verifier, test,
and Dockerfile also remain unchanged.

Exact evidence paths:

- `/tmp/lusi-assignment1-command-bridge/run.sh` — executed build/test commands
- `/tmp/lusi-assignment1-command-bridge/console.txt` — complete build and test output
- `/tmp/lusi-assignment1-command-bridge/test-result.txt` — final result summary
- `/tmp/lusi-assignment1-command-bridge/build/rover_sim_urc/test_results/rover_sim_urc/test_test_command_bridge_2027_launch.py.xunit.xml`
- `/tmp/lusi-assignment1-command-bridge/build/rover_sim_urc/launch_test/test_test_command_bridge_2027_launch.py.txt`
- `/tmp/lusi-assignment1-command-bridge/ros-logs/` — process logs
- `/tmp/lusi-assignment1-next-original.json` — this milestone's initial content hashes

### Remaining limits and mechanical gate

The four-wheel model itself has not been changed or driven by this bridge. Physical
hardware code, the 2027 message definition, and kinematics implementation are unchanged.
Full-rover steering integration stops here pending confirmed steering-axis locations
and orientations, linkage topology, travel limits, and agreement on actual geometry.
Electrical motor mappings, gearing, encoder conventions, and actuator dynamics would
also need confirmation before any physical control implementation.

This bridge intentionally rejects a normal six-actuator command with nonzero
unsupported fields. It is not a complete 2027 drivetrain controller. It trusts
receipt time rather than the message header; delayed/replayed messages are not
filtered. The watchdog is exercised under a running simulation, not during a paused
world or bridge process failure. Direct publishers can bypass the adapter and its
watchdog. Ideal simulated steering centering is not a verified physical emergency
stop strategy. Neither GUI rendering nor ground-contact locomotion was tested.
Existing headless display/audio and Gazebo Classic lifecycle warnings remain.

## Reliability milestone: bridge process failure containment — 2026-11-08

Before editing, the current tracked diff, untracked simulation implementation,
message definition, kinematics header, and prior numerical evidence were reviewed.
`docs/SOFTWARE_PROGRESS.md` **does not exist**; it was not created or substituted
for existing history. This dated section appends verified software progress here.

### Review and chosen gap

- Validation rejects unsupported actuator fields, nonfinite values, and commands
  outside the existing fixture's limits. Prior rejection tests remain unchanged.
- The 0.5 s receipt-time timeout uses monotonic time and a steady timer; it depends
  on the bridge process and its executor continuing to run.
- Normal shutdown publishes a final zero only while `rclpy.ok()` remains true.
  That publication is an attempt, not confirmation of controller receipt.
- An abrupt crash/SIGKILL bypasses shutdown code and removes the watchdog. The
  forward controllers can retain their last commanded motion while Gazebo continues.
- Stalled executors, old/replayed messages, and direct-controller publishers remain
  separate gaps; this change addresses process exit only.

The selected small improvement supervises the bridge at launch level. Its exit
requests shutdown of the isolated simulation. The success criterion was measured
motion at 2 rad/s and 0.3 rad steering, followed by killing only that bridge and
observing clean Gazebo termination within five seconds. Existing tests must pass.
No mechanical assumptions or physical hardware stop policies are introduced.

### Exact files changed in this milestone

- `src/rover_sim_urc/launch/assignment1_command_2027.launch.py` — register an
  `OnProcessExit` handler before starting the bridge; request launch-wide `Shutdown`.
- `src/rover_sim_urc/test/test_command_bridge_failure_launch.py` — new real Gazebo
  failure test with measured pre-failure motion, scoped SIGKILL, and shutdown checks.
- `src/rover_sim_urc/CMakeLists.txt` — register the new test under the existing
  opt-in `ROVER_SIM_TEST_2027_COMMAND_BRIDGE` option.
- `src/rover_sim_urc/README.md` — describe supervision scope, test, and limitations.
- `docs/ASSIGNMENT1_VERIFICATION.md` — append this dated evidence and review.

The bridge implementation, both existing test/verifier implementations, original
models/controllers/launch, Dockerfile, message definition, and kinematics remain
unchanged. Initial content hashes were recorded at
`/tmp/lusi-assignment1-supervision-original.json`; comparisons also confirmed all
eight files from the original uncommitted-work manifest remained unchanged.

### Actual tests and numerical evidence

Reused the approved `lusi-assignment1-sim` image as its non-root user, with source
directories mounted read-only. No new packages or images were installed/built.
Message and simulation packages built successfully in separate `/tmp` outputs.
Final tests reused copied build/install outputs after test-harness-only changes.
ROS domain 82 and Gazebo master `http://127.0.0.1:11382` isolated the final run.

```text
PASS: command=(2.00 rad/s, 0.30 rad), measured=(2.000 rad/s, 0.300 rad), wheel travel=1.000 rad
PASS: before bridge failure, wheel=2.000 rad/s, steering=0.300 rad, wheel position=1.020 rad
PASS: bridge SIGKILL caused clean Gazebo shutdown in 0.509s
1/3 Test #1: test_test_assignment1_launch.py .............. Passed 6.90 sec
2/3 Test #2: test_test_command_bridge_2027_launch.py ...... Passed 16.49 sec
3/3 Test #3: test_test_command_bridge_failure_launch.py ... Passed 6.55 sec
100% tests passed, 0 tests failed out of 3
Total Test time (real) = 29.95 sec
Summary: 6 tests, 0 errors, 0 failures, 0 skipped
```

The six reported results count three CTest wrappers and three underlying launch-test
results. The original message test also reverified command-silence stopping/centering
in 0.513 s and its existing invalid-command cases. The failure test deliberately
sends no cleanup stop command; termination must come from launch supervision.
Gazebo and the child launcher reported clean exits. The bridge's `exit code -9`
is the expected injected failure, not an unexpected crash of the final test.

Two initial test-harness failures were preserved:

1. `/tmp/lusi-assignment1-supervision/`: Gazebo shut down in 0.343 s, but the outer
   runner failed with `Launch stopped before the active tests finished.` The
   supervised launch was then isolated in a child LaunchService, with the outer
   test marked to stay alive until its assertions finish.
2. `/tmp/lusi-assignment1-supervision-r2/`: the supervisor again terminated Gazebo
   cleanly, but `Waiting for output timed out` because the assertion inspected
   stderr while the nested launch output was on stdout. Both assertions now
   explicitly inspect stdout. The production supervision code was unchanged.

Final passing evidence:

- `/tmp/lusi-assignment1-supervision-r3/run.sh`
- `/tmp/lusi-assignment1-supervision-r3/console.txt`
- `/tmp/lusi-assignment1-supervision-r3/test-result.txt`
- `/tmp/lusi-assignment1-supervision-r3/build/rover_sim_urc/test_results/rover_sim_urc/test_test_command_bridge_failure_launch.py.xunit.xml`
- `/tmp/lusi-assignment1-supervision-r3/build/rover_sim_urc/launch_test/test_test_command_bridge_failure_launch.py.txt`
- `/tmp/lusi-assignment1-supervision-r3/ros-logs/`

### Dependencies, limits, and next integration gate

The verified path depends on the generated `cross_pkg_messages/RoverDriveCommand2027`
type, the Gazebo ROS control plugin, velocity/position controllers, measured joint
feedback, and the launch supervisor staying alive. It does not integrate joystick,
navigation, CAN, science, arm, or perception subsystems; no new claims are made
about their status. No physical hardware control code changed.

The handler emits a **launch-wide** shutdown. This configuration is intended to run
in isolation; embedding it in a larger launch session extends that shutdown to
other processes in the session. The original direct-controller launch is unaffected.
This is tested simulation containment, not proof of measured zero joint velocity
after process death and not a physical emergency stop. No final zero-message delivery
guarantee was added. A bridge hang or supervisor failure can still leave commands
active; those require an independent downstream expiration/health mechanism.
The five-second bound is a test criterion, not a hard real-time shutdown guarantee.

The next software reliability dependency is a stop mechanism that survives a stalled
command producer. Full-rover steering integration still requires confirmed steering
axis/linkage geometry and travel limits; the provisional kinematics dimensions are
not promoted to hardware specifications. No commits, pushes, resets, or cleaning
were performed.

## Independent downstream command gate — 2026-11-08

This dated addition records the gate implementation after approval. Prior sections
remain historical evidence; the previously open bridge-hang gap is now covered in
the two-joint simulation while the independent gate remains operational.

### Reviewed baseline and implementation

Before editing, the tracked diff, existing bridge, unchanged message definition,
launch configuration, controller YAML, and all three integration tests were reviewed.
Initial hashes are at `/tmp/lusi-assignment1-gate-original.json`.

The production command path is now:

```text
/assignment1/rover_drive_command_2027
  -> command_bridge_2027.py (existing validation and upstream timeout)
  -> /assignment1/validated_drive_command_2027 (RoverDriveCommand2027)
  -> command_gate_2027.py (independent validation and receipt-time expiration)
  -> existing wheel velocity / steering position forward controllers
  -> Gazebo joints -> measured JointState feedback
```

The gate is a separate process and the only controller-command publisher created
by this launch. Every envelope is independently checked for finite values, zero
unsupported fields, and the existing fixture limits (±6 rad/s, ±0.6 rad). Valid
reception renews a monotonic 0.5 s lease. A steady-clock timer checks every 50 ms;
controller output is also updated immediately on incoming envelopes. Expiration
clears cached motion and repeatedly sends zero velocity/zero fixture steering.
Invalid envelopes clear the lease and request zero immediately; they do not renew
it. Repeating controller output does not renew the input lease. Fresh valid input
recovers motion. Startup cache values and output are zero.

The bridge no longer creates either controller publisher. It sends an atomic pair
in an existing message envelope. Its original timeout and invalid-input handling
remain, now producing valid zero envelopes rather than direct-controller output.
An invalid upstream request cannot sustain motion through that zero envelope.
Invalid envelopes sent directly to the gate are independently rejected.
Both bridge and gate exits request shutdown of the isolated launch.

### Exact changed files and test adaptations

Added:

- `src/rover_sim_urc/scripts/command_gate_2027.py`
- `src/rover_sim_urc/test/test_command_gate_launch.py`

Modified:

- `src/rover_sim_urc/scripts/command_bridge_2027.py` — validated-envelope output.
- `src/rover_sim_urc/launch/assignment1_command_2027.launch.py` — start/supervise gate.
- `src/rover_sim_urc/CMakeLists.txt` — install gate and register its integration test.
- `src/rover_sim_urc/scripts/verify_assignment1.py` — optional direct publishers;
  default remains enabled for the original direct-controller test.
- `src/rover_sim_urc/scripts/verify_command_bridge_2027.py` — disable inherited unused
  direct publishers so typed-message tests do not violate sole output ownership.
- `src/rover_sim_urc/test/test_command_bridge_failure_launch.py` — extend the scoped
  PID helper with an optional executable name; its default still selects the bridge.
- `src/rover_sim_urc/README.md` — current command path, operation, and limitations.
- `docs/ASSIGNMENT1_VERIFICATION.md` — append this evidence.

These test adaptations were explained before editing. All existing positive/reverse
motion, displacement, finite feedback, rejection timing, silence timeout, and
bridge SIGKILL/clean Gazebo shutdown assertions remain. The three original test
cases ran again as regressions; the fourth test exercises gate-specific failures.
The new test injects invalid envelopes while the bridge is SIGSTOP-suspended,
preventing upstream sanitization or its watchdog from masking gate behavior.

### Actual build and measured results

Used the existing approved `lusi-assignment1-sim` image as its non-root user, with
both source packages mounted read-only. No packages, working image tags, Docker
configurations, or original build caches were changed. New build/install/log
artifacts are isolated under `/tmp/lusi-assignment1-command-gate`. ROS domain 84,
Gazebo master `http://127.0.0.1:11384`, Humble, and Gazebo 11.10.2 were used.

```text
Finished <<< cross_pkg_messages [13.2s]
Summary: 1 package finished [13.8s]
Finished <<< rover_sim_urc [1.68s]
Summary: 1 package finished [2.01s]
```

| Check | Actual measured result |
|---|---|
| Startup without upstream user commands | 0.80 s stopped, 80 fresh samples, 0.000000 rad travel; repeated zero output on both topics |
| Sole output ownership | Exactly one publisher on each controller topic: `assignment1_command_gate_2027` |
| Motion before bridge suspension | 2.000 rad/s wheel, 0.300 rad steering, 1.020 rad wheel travel |
| Bridge SIGSTOP expiration | Measured stop/centering in 0.552 s; bridge remained stopped and supervisor alive |
| Sustained stop after expiration | 1.00 s, 100 fresh samples, negligible wheel travel; 18 observed zero messages per topic after excluding first 0.1 s |
| Gate-input silence, bridge still suspended | Measured stop/centering in 0.543 s after valid reverse input ceased |
| Sustained stop after gate-input silence | 1.00 s, 100 samples, 0.000000 rad travel; 18 observed zero messages per topic after initial 0.1 s |
| Fresh upstream recovery | -1.500 rad/s wheel, -0.250 rad steering, -0.750 rad wheel travel |
| Bridge SIGKILL regression | Clean Gazebo shutdown in 0.426 s |
| Gate SIGKILL while moving | Clean Gazebo shutdown in 0.462 s |

Startup tested absence of upstream user commands; the bridge itself emits valid
zero envelopes at startup. The gate also continued sending zero after its input
ceased. No stronger claim of a completely silent initial envelope topic is made.

Independent gate rejection results, each measured before its 0.5 s timeout:

| Invalid envelope field/value | Measured stop/centering (s) |
|---|---:|
| `rear_left_velocity=1.0` | 0.020 |
| `front_right_velocity=1.0` | 0.020 |
| `rear_right_velocity=1.0` | 0.019 |
| `front_right_steering=0.1` | 0.021 |
| `front_left_velocity=NaN` | 0.019 |
| `front_left_steering=Inf` | 0.020 |
| `front_left_velocity=6.1` | 0.030 |
| `front_left_steering=0.61` | 0.020 |
| `front_left_velocity=-6.1` | 0.020 |
| `front_left_steering=-0.61` | 0.019 |

A further one-second invalid-input stream produced 100 stopped feedback samples,
zero wheel travel, and repeated zero output. This test's output count exceeds the
timer rate because incoming rejected envelopes also cause immediate zero output.

```text
1/4 Test #1: test_test_assignment1_launch.py .............. Passed 6.75 sec
2/4 Test #2: test_test_command_bridge_2027_launch.py ...... Passed 17.09 sec
3/4 Test #3: test_test_command_bridge_failure_launch.py ... Passed 6.09 sec
4/4 Test #4: test_test_command_gate_launch.py ............. Passed 24.23 sec
100% tests passed, 0 tests failed out of 4
Total Test time (real) = 54.15 sec
Summary: 8 tests, 0 errors, 0 failures, 0 skipped
```

Four integration tests produced eight result entries because colcon also counts
the CTest wrappers. **No build or test failures occurred during this implementation;
the complete suite passed on its first execution.** Intentional SIGKILL diagnostics
for the bridge and gate are expected fault injections. Earlier milestones' corrected
test-harness failures remain documented in their original sections.

Evidence paths:

- `/tmp/lusi-assignment1-command-gate/run.sh`
- `/tmp/lusi-assignment1-command-gate/console.txt`
- `/tmp/lusi-assignment1-command-gate/test-result.txt`
- `/tmp/lusi-assignment1-command-gate/build/rover_sim_urc/test_results/rover_sim_urc/test_test_command_gate_launch.py.xunit.xml`
- `/tmp/lusi-assignment1-command-gate/build/rover_sim_urc/launch_test/test_test_command_gate_launch.py.txt`
- `/tmp/lusi-assignment1-command-gate/ros-logs/`

### Preservation and remaining limitations

Existing four-/six-wheel models, test URDF/world, controller configuration, physical
hardware code, provisional kinematics, message definition, package dependencies,
and Dockerfile are unchanged. Original uncommitted work was retained. No commits,
pushes, resets, or cleaning occurred. The canonical `docs/SOFTWARE_PROGRESS.md`
remains absent and was not created.

**There is no replay protection.** The lease measures receipt time, not semantic
freshness. Repeated old valid envelopes, delayed buffered envelopes, or another
publisher retransmitting stale values can renew it and command motion. Headers
and sequence ordering are not validated. Depth-one queues do not resolve this.

The gate must remain responsive. A gate hang, controller update failure, supervisor
failure, and competing direct-controller publishers remain outside the verified
protection. Sole output ownership was verified for this launch/test graph, not
enforced by DDS permissions. Exit supervision is launch-wide and must stay isolated.
Neither paused-world behavior nor physical braking is verified. Zero steering is
the fixture's existing policy, and measured timing is not a hard real-time bound.
Whole-rover steering still requires confirmed mechanical specifications.

### Additional startup coverage and final five-test run

The first gate test established startup without upstream user requests, but the
bridge still supplied zero envelopes. To independently test the gate's initial
state with no input at all, added
`src/rover_sim_urc/test/test_command_gate_startup_launch.py` and registered it under
the existing opt-in CMake flag. It starts the unchanged fixture and real gate only.
It asserts zero publishers on `/assignment1/validated_drive_command_2027` both
before and after observation. Actual result: **100 fresh stopped samples over
1.0 s and 20 zero outputs per controller topic**. No bridge or envelope source
exists in that startup test graph.

All five tests then ran serially in a second isolated directory, preserving the
first passing run. Production source was unchanged; the simulation package was
rebuilt to register the new test, reusing copied build/install outputs.
ROS domain 85 and Gazebo master `http://127.0.0.1:11385` isolated the final suite.

Final measurements:

- Bridge SIGSTOP: stop/centering in **0.542 s**, wheel velocity approximately
  `-1.36e-10 rad/s` and steering approximately `-9.73e-11 rad` after stopping.
- Independent gate-input silence: stop/centering in **0.558 s**.
- Bridge SIGKILL regression: clean simulation shutdown in **0.377 s**.
- Gate SIGKILL: clean simulation shutdown in **0.508 s**.
- Sole gate output ownership, ten independent rejection cases, sustained stopped
  feedback, repeated zero output, and fresh-command recovery passed again.

```text
1/5 Test #1: test_test_assignment1_launch.py .............. Passed 8.21 sec
2/5 Test #2: test_test_command_bridge_2027_launch.py ...... Passed 17.86 sec
3/5 Test #3: test_test_command_bridge_failure_launch.py ... Passed 8.64 sec
4/5 Test #4: test_test_command_gate_launch.py ............. Passed 26.66 sec
5/5 Test #5: test_test_command_gate_startup_launch.py ..... Passed 6.25 sec
100% tests passed, 0 tests failed out of 5
Total Test time (real) = 67.62 sec
Summary: 10 tests, 0 errors, 0 failures, 0 skipped
```

There are five integration tests, with their CTest wrappers also counted by colcon.
**Both suite executions passed; no build or test failures were corrected during
this gate milestone.** Numerical differences between runs are observed scheduling
and simulation timing, not hard real-time guarantees.

Final evidence:

- `/tmp/lusi-assignment1-command-gate-r2/run.sh`
- `/tmp/lusi-assignment1-command-gate-r2/console.txt`
- `/tmp/lusi-assignment1-command-gate-r2/test-result.txt`
- `/tmp/lusi-assignment1-command-gate-r2/build/rover_sim_urc/test_results/rover_sim_urc/test_test_command_gate_launch.py.xunit.xml`
- `/tmp/lusi-assignment1-command-gate-r2/build/rover_sim_urc/test_results/rover_sim_urc/test_test_command_gate_startup_launch.py.xunit.xml`
- `/tmp/lusi-assignment1-command-gate-r2/ros-logs/`

The full changed-file list for this milestone is the two added files and eight
modified files above, plus this added startup test (eleven repository files).
All protection and replay limitations in the preceding section still apply.

## Four existing wheel-drive joints — 2026-11-08

### Baseline and review

Clean-checkout reproduction v3 preceded this milestone and passed all five
original integration tests in 52.92 s, with zero errors/failures/skips (colcon
counts ten entries including wrappers). Evidence remains unchanged at
`/home/rek429/urc-reproduction/assignment1-20261108-r1/output/`.
Git HEAD remains `1686e21f82d6cfd9994370b4d7876684569053fd`, branch
`feature/2027-drivetrain-migration`; all work remains uncommitted.

Reviewed both progress documents, message, original SDF, controller YAML,
bridge/gate, provisional kinematics, and test evidence. GitHub returned no open
PRs; latest retrieved changes were the science timing fix and state-machine plan:
https://github.com/Lehigh-University-Space-Initiative/urc_software/commit/936e740d64e9418b1eef04e9d946d2a7617f0e6c
and https://github.com/Lehigh-University-Space-Initiative/urc_software/commit/85444bea02877defd9d993aed33fb82d76020461.
Confluence Rovo search `URC drivetrain Gazebo` returned no matching pages.
An initial search request used limit=8 and was rejected by connector schema;
limit=10 succeeded. No internal absence-of-conflicts guarantee follows.

### Separate configuration and exact changes

Added relative to `src/rover_sim_urc/`:

- `launch/four_wheel.launch.py`
- `config/four_wheel_controllers.yaml`
- `scripts/four_wheel_bridge.py`
- `scripts/four_wheel_gate.py`
- `test/test_four_wheel_launch.py`

Modified `CMakeLists.txt` only to install the new executables and register the
new opt-in `ROVER_SIM_TEST_FOUR_WHEEL` test. Added the already available
`controller_manager_msgs` test dependency in `package.xml` after the suite run;
validated the resulting XML. No runtime implementation changed after that run.
Appended this document and `docs/SOFTWARE_PROGRESS.md`.

The launch derives a separate suspended URDF from the unchanged
`models/lusi_rover/model.sdf`. A structural comparison verified joint names,
parent/child links, axes, wheel placements, masses, inertias and geometry sizes.
It adds a world mount at 0.65 m, ROS interfaces, and simulation-only effort=20
and velocity=6 caps. The SDF model-level placement is replaced by the stand mount;
visual materials are not transferred. The original SDF is not directly spawned.
This is a controlled derivative, not a new validated physical model.

The four original revolute joints attach directly to chassis; axes are `0 1 0`.
There are no steering joints. Controller/message array order is explicitly:

1. `front_left_wheel_joint` / `front_left_velocity`
2. `rear_left_wheel_joint` / `rear_left_velocity`
3. `front_right_wheel_joint` / `front_right_velocity`
4. `rear_right_wheel_joint` / `rear_right_velocity`

The typed command path is under `/assignment1_four_wheel`:
`rover_drive_command_2027` -> bridge -> `validated_command` -> gate ->
`wheel_velocity_controller/commands` -> four Gazebo joints ->
`joint_state_broadcaster/joint_states`.
Both steering fields must be exactly zero. Both processes independently reject
nonfinite fields, steering requests, and wheel speeds outside ±6 rad/s.
These are simulation caps, not approved hardware limits.
The gate is the sole configured controller-command publisher. Its independent
0.5 s monotonic lease is checked every 50 ms; invalid input clears the lease,
expiration repeatedly publishes four zeros, and fresh input restores commands.
Both command-process exits request isolated launch-wide shutdown.

### Actual build and serial integration results

Used pinned image
`sha256:f2b95edaa669f69a6a785dbff0187cfeb61ad950d77962b58b2dd5dead62c603`,
ROS 2 Humble / Gazebo Classic 11.10.2, non-root `lusi_sim`, read-only root and
repository mount, explicit writable runtime directories, ROS domain 87 and
Gazebo master `http://127.0.0.1:11387`. No image build or dependency installation.
Fresh message/simulation build, install and log directories were used.

Both packages built. Build options were `BUILD_TESTING=ON`,
`ROVER_SIM_TEST_2027_COMMAND_BRIDGE=ON`, `ROVER_SIM_TEST_FOUR_WHEEL=ON`.
`colcon test` used `--ctest-args -j1`; exact build/test commands are retained in
`/tmp/lusi-assignment1-four-wheel/run.sh`.

| Test file | CTest duration | Result |
|---|---:|---|
| test_assignment1_launch.py | 4.99 s | Passed unchanged |
| test_command_bridge_2027_launch.py | 16.11 s | Passed unchanged |
| test_command_bridge_failure_launch.py | 4.21 s | Passed unchanged |
| test_command_gate_launch.py | 23.58 s | Passed unchanged |
| test_command_gate_startup_launch.py | 4.47 s | Passed unchanged |
| test_four_wheel_launch.py | 23.78 s | Passed |

Total CTest time: 77.14 s. Colcon: **12 tests, 0 errors, 0 failures, 0 skipped**
(six integration results plus wrappers). XML verification required exactly six
reports with one test each and no errors, failures, or skips.
**No build or integration-test failures occurred in this milestone.**
Expected SIGKILL error diagnostics are intentional injected faults.

Measured four-joint results, ordered as above:

- Each wheel individually: +1.500 and -1.500 rad/s; inactive wheels remained
  zero. Active signed displacement over the observation interval was
  +0.750 to +0.765 rad or -0.750 rad; inactive displacement rounded to zero.
- Coordinated forward: `[2.000, 2.000, 2.000, 2.000]` rad/s;
  initial observation displacement `[1.020, 1.020, 1.020, 1.020]` rad.
- Coordinated reverse: `[-1.500, -1.500, -1.500, -1.500]` rad/s;
  initial displacement `[-0.750, -0.750, -0.750, -0.750]` rad.
- Explicit zero: all four velocities/displacements rounded to zero.
- Upstream command silence: measured stop in 0.482 s after publishing stopped.
  This measures time from test transmission cessation, not last gate reception.
- Bridge SIGSTOP: independent measured stop in 0.503 s; one second of sustained
  stopping, 100 fresh samples, negligible travel, repeated four-zero outputs.
- Gate-input silence with bridge still stopped: measured stop in 0.550 s;
  101 samples over the following one-second stopped interval.
- Fresh upstream commands after SIGCONT restored all four to -1.500 rad/s.
- Six direct gate rejection cases: FL velocity NaN, RL velocity Inf,
  FR velocity +6.1, RR velocity -6.1, FL steering +0.1, FR steering -0.1.
  Measured stopping **plus a 0.3 s stable observation** completed in
  0.329, 0.326, 0.320, 0.329, 0.322 and 0.322 s respectively, all before timeout.
  These values are not pure stop latencies. Upstream rejection of RR NaN and
  both nonzero steering fields also produced measured stopping.
- Both controllers were confirmed active via ListControllers; exactly one
  command publisher was observed, named `four_wheel_gate`.
- Gate SIGKILL while all four measured 2 rad/s caused clean Gazebo shutdown
  in 0.340 s. This verifies containment, not measured stopping after death.

### Evidence and limitations

All evidence from this run remains at:

- `/tmp/lusi-assignment1-four-wheel/run.sh`
- `/tmp/lusi-assignment1-four-wheel/model-parity.txt`
- `/tmp/lusi-assignment1-four-wheel/output/console.txt`
- `/tmp/lusi-assignment1-four-wheel/output/test-result.txt`
- `/tmp/lusi-assignment1-four-wheel/output/build/rover_sim_urc/test_results/rover_sim_urc/`
- `/tmp/lusi-assignment1-four-wheel/output/build/rover_sim_urc/launch_test/`
- `/tmp/lusi-assignment1-four-wheel/output/{message-build-log,sim-build-log,test-log,ros-logs,gazebo-logs}/`
- `/tmp/lusi-four-wheel-baseline.json` and
  `/tmp/lusi-assignment1-original-work.json` (preservation manifests).

Temporary artifacts require a separate durable preservation step. The earlier
verified evidence bundle and reproduction workspace were not changed.
Warnings include headless rendering/audio availability, Gazebo Classic lifecycle,
100 Hz control versus 1000 Hz simulation, and joint-state interface mapping.
No isolation/shared-memory workaround was needed.

Original models, message definition, provisional kinematics, physical control
code, two-joint implementation, and all five original tests remain unchanged.
All eight original preexisting file hashes were checked against their manifest.

This demonstrates suspended joint rotation only. No ground contact, chassis
locomotion, steering, odometry, motor calibration, or physical braking is verified.
Repeated stale envelopes still renew leases; gate hangs remain uncontained.
Competing publishers, DDS loss/asymmetry, controller-manager failures, paused
worlds, resets, long runs, and exact limit boundaries remain untested.
Four-wheel bridge-exit supervision is configured but was not separately fault
injected; original two-joint bridge-exit regression passed.
Geometry and limits remain simulation data. Mechanical steering specifications
and Electrical motor/encoder mapping are still required before hardware work.
No commits, staging, pushes, merges, resets, cleaning or existing resource
removal occurred.

## Independent gate-health monitoring — 2026-11-08

### Scope and implementation

Reviewed Git status, current four-wheel gate/bridge, controller YAML, launch
supervision, six existing tests, and preserved six-test evidence before editing.
Both checksum manifests passed at
`/home/rek429/urc-evidence/assignment1-four-wheel-20261108-r1`.
The original evidence bundle remains unchanged.
Baseline hashes: `/tmp/lusi-gate-health-baseline.json`.

The four-wheel gate now publishes `std_msgs/UInt64` progress on
`/assignment1_four_wheel/gate_progress` **after** each controller publish returns.
The value is `time.monotonic_ns()` from the same Linux clock domain.
This indicates callback progress, not DDS delivery or actuator acknowledgement.
Its command validation, expiration, and four-wheel output remain unchanged.

A separate `gate_health_monitor.py` process accepts only nonzero, nonfuture,
strictly increasing heartbeat timestamps less than 0.5 seconds old.
Repeated/equal, older, expired and future values cannot renew the lease.
The deadline is based on the accepted timestamp, rather than its reception time.
A steady timer checks every 50 ms, independently of `/clock`.

- Startup without a heartbeat: terminal failure after three monotonic seconds.
- After a first accepted heartbeat: terminal failure at timestamp + 0.5 s.
- A callback arriving after expiration cannot revive the monitor.
- On expiration the monitor logs the reason and exits with status 2.
- Its `OnProcessExit` handler triggers the existing launch-wide shutdown.
- The monitor has no controller-command publisher.
- A SIGSTOP gate cannot handle SIGINT/SIGTERM; its launch action now uses
  0.5 s SIGTERM and 0.5 s SIGKILL escalation intervals.

The startup grace is bounded but does not interlock initial actuator commands.
These deadlines require scheduling of the monitor and launch supervisor and
are not hard real-time guarantees. The clock assumption is same-host processes;
this heartbeat scheme is not suitable for independently clocked remote hosts.

### Exact file changes

Added relative to `src/rover_sim_urc/`:

- `scripts/gate_health_monitor.py`
- `test/test_gate_health_monitor_launch.py`
- `test/test_gate_health_failure_launch.py`

Modified:

- `scripts/four_wheel_gate.py`: progress publisher after existing command output;
  updated scope comment, no command-policy change.
- `launch/four_wheel.launch.py`: start/supervise monitor and bound stopped-gate
  signal escalation; model conversion and controllers remain unchanged.
- `CMakeLists.txt`: install monitor and register two tests under existing
  `ROVER_SIM_TEST_FOUR_WHEEL` opt-in.
- `docs/ASSIGNMENT1_VERIFICATION.md` and `docs/SOFTWARE_PROGRESS.md`: append evidence.

All six original test files and assertions, both original rover models,
message definition, four-wheel bridge, controller configurations, physical
hardware code, and provisional kinematics remain unchanged.
No new packages, image builds, commits, staging, resets, or cleaning.

### Environment and final passing results

Reused image
`sha256:f2b95edaa669f69a6a785dbff0187cfeb61ad950d77962b58b2dd5dead62c603`,
Humble/Gazebo Classic 11.10.2, non-root user, read-only root/source mounts,
explicit runtime mounts and fresh build/install/log directories.
Initial run used domain 88, Gazebo port 11388; final run domain 89, port 11389.
Build flags remained `BUILD_TESTING=ON`, `ROVER_SIM_TEST_2027_COMMAND_BRIDGE=ON`,
`ROVER_SIM_TEST_FOUR_WHEEL=ON`. Tests ran serially with `--ctest-args -j1`.
Exact build/test commands are retained in each run's `run.sh`.

| Test | Final CTest duration | Result |
|---|---:|---|
| test_assignment1_launch.py | 4.91 s | Passed unchanged |
| test_command_bridge_2027_launch.py | 15.30 s | Passed unchanged |
| test_command_bridge_failure_launch.py | 4.60 s | Passed unchanged |
| test_command_gate_launch.py | 22.18 s | Passed unchanged |
| test_command_gate_startup_launch.py | 4.76 s | Passed unchanged |
| test_four_wheel_launch.py | 24.08 s | Passed unchanged |
| test_gate_health_monitor_launch.py | 5.53 s | Passed |
| test_gate_health_failure_launch.py | 8.22 s | Passed |

Final CTest total: **89.59 s**. Colcon: **16 tests, 0 errors, 0 failures,
0 skipped**, counting eight underlying tests and their wrappers.
Exactly eight one-test XML reports were required, all without errors/failures/skips.

Measured new-test results:

- Real gate/monitor under Gazebo: all four joints measured 2.000 rad/s before
  gate SIGSTOP; 201 distinct progress samples observed by the probe.
- SIGSTOP to deadline observation: **0.552 s**.
- SIGSTOP to clean Gazebo exit: **0.990 s**.
- SIGSTOP to full launch exit: **1.794 s**, including SIGKILL of the stopped gate.
- Acceptance bounds: detection <1 s, clean Gazebo exit <3 s, full launch exit <5 s.
- Standalone real monitor processes used `use_sim_time=true`, with no `/clock`
  source launched. Advancing progress sustained healthy operation for four seconds.
- No-heartbeat startup process exit observed **3.557 s from test-loop start**;
  this includes monitor/CLI startup overhead, not an exact internal timer reading.
- Constant stale heartbeat repeatedly published: first transmission to monitor/CLI
  exit **0.696 s**. Repetition did not renew its deadline.
- Fresh heartbeat publication cessation to monitor/CLI exit: **0.695 s**.
  Exit observations include ROS CLI/process shutdown overhead.
- All three standalone expiration processes exited with status 2.

The original four-wheel regression reverified sole command-publisher ownership,
individual/coordinated signed motion, stopping, invalid input, bridge suspension,
command loss, recovery and gate exit. The monitor adds no controller publisher.

### Failures and runtime exceptions, including passing-run limitations

The first full suite is retained at `/tmp/lusi-assignment1-gate-health/`.
It took 90.55 s: six tests passed and two did not.
Colcon counted 16 entries with **1 error, 3 failures, 0 skipped** including wrappers.

1. Unchanged two-joint bridge-exit regression: after intentional bridge SIGKILL,
   Gazebo aborted (exit -6) with
   `boost thread: trying joining itself: Resource deadlock avoided`
   (`boost::thread_resource_error`). The clean-Gazebo-exit assertion timed out.
   No original implementation/test/assertion was changed to accommodate it.
   The same regression passed on the final run; the shutdown self-join issue
   is not root-caused or fixed by this milestone.
2. New standalone monitor harness: accessed `returncode` on launch_testing's
   `ProcessStarted` event and raised AttributeError. Corrected only that new
   test to treat a running event without the field as not yet exited.
   The failed-run source snapshot is retained under its `source/` directory.

The new Gazebo SIGSTOP test already passed in the first run: detection 0.533 s,
Gazebo exit 0.950 s, full launch exit 1.801 s.
The second full suite used new directories and passed after the harness correction.

The passing run also contains a real **bridge shutdown exception** in test 8:
its unchanged `run()` final zero publish raced ROS context shutdown, producing
`RCLError: Failed to publish: publisher's context is invalid` and bridge exit 1.
The monitor had already requested shutdown; Gazebo still exited cleanly and
containment bounds passed. This is not classified as an intentional injected
bridge failure and is not fixed. Tests verify containment, not clean exit of
every child. The stopped gate's signal escalation and final exit -9 are expected.
Normal headless audio/display, Gazebo lifecycle, control-period and interface
mapping warnings remain. No isolation or shared-memory workaround was used.

### Evidence, preservation and remaining risks

Final passing evidence: `/tmp/lusi-assignment1-gate-health-r2/`.
Failed/initial evidence: `/tmp/lusi-assignment1-gate-health/`.
Each includes `run.sh`, source hashes, console, test-result summary, eight XML
reports, launch output, ROS/Gazebo logs and fresh build/test logs under `output/`.
Final source snapshot and checksums are recorded alongside the output artifacts.
These are temporary paths pending a separately approved durable preservation step.

All eight original preexisting file hashes and all six existing test hashes
remain unchanged. Git HEAD/branch/stash remain preserved. Existing durable
six-test evidence, original backups, models and Docker images were not modified.

This verifies **simulation shutdown after loss of gate progress**, not wheel
braking or measured zero motion after gate suspension. Stale drive-command replay
still renews the gate lease and can coexist with a healthy progress heartbeat.
Forged advancing heartbeat publishers, direct controller publishers, monitor or
supervisor hangs, controller-manager/DDS faults and feedback loss are uncontained.
No steering, ground contact, hardware control or kinematics were added.

Next: preserve both runs, then review shutdown reliability (the Gazebo self-join
abort and bridge final-publish/context race) before extending safety claims or
starting ground-contact development. Hardware/steering prerequisites remain open.

## ROS 2 shutdown cleanup correction — 2026-11-08

### Reviewed scope and narrow correction

Before editing, inspected Git status, `scripts/four_wheel_bridge.py`,
`scripts/four_wheel_gate.py`, monitor, launch handlers, and preserved eight-test
results at `/home/rek429/urc-evidence/assignment1-gate-health-20261108-r1`.
Both bundle checksum manifests passed. Baseline file hashes are retained at
`/tmp/lusi-cleanup-baseline.json`.

The shared `run()` helper formerly checked `rclpy.ok()` and then attempted a final
zero publication. An asynchronous signal-triggered ROS context shutdown could
invalidate that context between the check and publication. The approved correction:

- Removes all teardown publication; cleanup never invokes the node's tick callback.
- Handles `ExternalShutdownException` as expected executor termination, alongside
  KeyboardInterrupt. Other runtime failures continue to propagate.
- Constructs the node inside the cleanup-protected try block.
- Uses a shared `cleanup(node)` function: attempt node destruction, then always
  call idempotent `rclpy.try_shutdown()` for the default context owned by `run()`.
- Cleans the initialized context even if construction or destruction fails.
- Leaves all running command callbacks, validation, timers, expiration and
  heartbeat publications unchanged.

Both FourWheelBridge and FourWheelGate import/use this helper. The gate's own
source did not change. Cleanup is scoped to the default-context standalone
processes; it is not a general multi-context lifecycle manager.
No final zero delivery is promised. Independent command expiration and launch
containment remain responsible for the verified simulation behavior.

### Exact files changed

Modified:

- `src/rover_sim_urc/scripts/four_wheel_bridge.py`: shared cleanup/run helper only.
- `src/rover_sim_urc/CMakeLists.txt`: register two new tests under the existing
  `ROVER_SIM_TEST_FOUR_WHEEL` option.
- `docs/ASSIGNMENT1_VERIFICATION.md` and `docs/SOFTWARE_PROGRESS.md`: append evidence.

Added:

- `src/rover_sim_urc/test/test_ros_cleanup_launch.py`
- `src/rover_sim_urc/test/test_four_wheel_shutdown_launch.py`

All eight prior test files/assertions, controller configuration, gate, monitor,
launch files including Gazebo signal order/escalation intervals, original models,
message definition, hardware code and provisional kinematics remain unchanged.
No package installation, image build, commit, staging, reset or cleaning.

### Actual tests and measurements

Pinned image:
`sha256:f2b95edaa669f69a6a785dbff0187cfeb61ad950d77962b58b2dd5dead62c603`.
Humble / Gazebo Classic 11.10.2, non-root user, read-only root/source mounts,
explicit runtime mounts, domain 90, Gazebo master `http://127.0.0.1:11390`.
Fresh build/install/log directories under `/tmp/lusi-assignment1-ros-cleanup/`.
Message package built in 7.57 s and simulation package in 1.23 s.
Build flags: `BUILD_TESTING=ON`, `ROVER_SIM_TEST_2027_COMMAND_BRIDGE=ON`,
`ROVER_SIM_TEST_FOUR_WHEEL=ON`. Tests ran serially (`--ctest-args -j1`).
Exact build/test commands are retained in the run's `run.sh`.

| Test file | CTest duration | Result |
|---|---:|---|
| test_assignment1_launch.py | 6.91 s | Passed unchanged |
| test_command_bridge_2027_launch.py | 15.49 s | Passed unchanged |
| test_command_bridge_failure_launch.py | 4.36 s | Passed unchanged |
| test_command_gate_launch.py | 22.36 s | Passed unchanged |
| test_command_gate_startup_launch.py | 4.07 s | Passed unchanged |
| test_four_wheel_launch.py | 25.74 s | Passed unchanged |
| test_gate_health_monitor_launch.py | 5.33 s | Passed unchanged |
| test_gate_health_failure_launch.py | 5.87 s | Passed unchanged |
| test_ros_cleanup_launch.py | 0.67 s | Passed |
| test_four_wheel_shutdown_launch.py | 14.35 s | Passed |

CTest total: **105.14 s; 10/10 passed**. Colcon: **20 tests, 0 errors,
0 failures, 0 skipped**, including ten wrapper entries. Independent XML checks
required exactly ten one-test reports with no errors, failures or skipped cases.
**No build or test failures occurred in this correction milestone.**
The historical failures in the preceding milestone remain recorded and preserved.

Focused cleanup coverage used actual ROS nodes and contexts for both bridge and
gate, intercepting spin only to force context invalidation and
ExternalShutdownException. It verified no teardown tick/publication, harmless
repeated cleanup, context cleanup despite destruction failure, propagation of
non-shutdown runtime/destruction errors, and cleanup after constructor failure.

The new Gazebo test launches the unchanged production configuration serially for
three scenarios, confirming both controllers active and actual joint feedback:

| Scenario | Measured velocities before shutdown (rad/s) | Full shutdown |
|---|---|---:|
| Normal SIGINT to launcher, idle | Approximately [0, 0, 0, 0] | 0.366 s |
| Normal SIGINT to launcher, moving | [2.000, 2.000, 2.000, 2.000] | 0.366 s |
| Gate SIGSTOP causing monitor shutdown | [2.000, 2.000, 2.000, 2.000] | 1.876 s |

All three launchers exited 0. Bridge clean exit and clean Gazebo exit were
required in each scenario. Normal SIGINT also required clean gate exit; the
monitor scenario required the deliberately suspended gate's existing SIGKILL
containment (-9). All three required no traceback, invalid-context publication
message, or repeated-shutdown error. Those assertions passed. No production
Gazebo signal order or escalation change was made.

The separate unchanged gate-health test measured SIGSTOP detection in **0.543 s**,
clean Gazebo exit in **0.941 s**, and full launch exit in **1.788 s**.
The four-wheel regression measured stopping after upstream silence in **0.511 s**,
bridge SIGSTOP in **0.553 s**, and gate-input silence in **0.473 s** from the test's
transmission cessation/suspension points, not exact last-reception timestamps.
Individual/coordinated signed joint motion, invalid input, sustained stops,
recovery and sole gate controller-publisher ownership all passed unchanged.
The monitor again rejected repeated stale progress and expired without /clock.

### Evidence and limits

Evidence root: `/tmp/lusi-assignment1-ros-cleanup/`.

- `run.sh`: exact build/test commands; `source-before.json`: tested source hashes.
- `output/console.txt`, `output/test-result.txt`.
- `output/build/rover_sim_urc/test_results/rover_sim_urc/`: ten XML reports.
- `output/build/rover_sim_urc/launch_test/`: full test output.
- `output/cleanup-scenarios/{idle,moving,monitor}.txt`: complete per-scenario output.
- `output/{message-build-log,sim-build-log,test-log,ros-logs,gazebo-logs}/`.
- Source snapshot and SHA256SUMS retained alongside artifacts after verification.

The main console and all three scenario logs contained **zero tracebacks** and
no invalid-context publication errors. Expected injected bridge/gate SIGKILL,
monitor expiry status 2, headless graphics/audio, lifecycle, controller-period,
and interface-mapping diagnostics remain. No isolation/shared-memory failure.

All eight original preexisting file hashes and all eight existing test hashes
remain unchanged. Source files match the hashes frozen before this run.
Previously preserved evidence and repository history were not rewritten.
Temporary evidence still needs separately approved durable preservation.

This corrects the identified teardown publication path and verifies the tested
normal/monitor shutdown cases. It does not prove absence of every possible
signal race in active callbacks or all ROS components. Unexpected live errors
are not broadly suppressed. The historical Gazebo Boost self-join abort did not
recur in this run but is neither diagnosed nor fixed by this correction.
Simulation shutdown is not measured wheel braking. Replay, competing publishers,
monitor/supervisor hangs, controller/DDS faults, ground contact and physical
hardware remain outside the verified scope.

Next: preserve this ten-test evidence, then pursue the separately proposed
native-backtrace investigation of the historical Gazebo abort before changing
shutdown ordering or extending simulation safety claims.
