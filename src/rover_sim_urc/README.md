# Assignment 1: one wheel and one front steering joint

This ROS 2 Humble / Gazebo Classic 11 experiment suspends a front-left wheel
assembly from a fixed chassis. It checks actuator motion independently of rover
locomotion, ground contact, joystick mapping, and CAN hardware.

The existing four-wheel `models/lusi_rover/model.sdf` and six-wheel
`models/lusi_rover_6wheel_backup/model.sdf` remain available and unchanged.
The test assembly reuses the four-wheel model's front-left position, wheel radius,
width, inertia, and wheel joint name. These are provisional simulation dimensions.
The existing 2027 kinematics uses a different radius (0.108 m); reconcile dimensions
with mechanical CAD before integrating whole-rover commands.

## Run in an existing Humble development environment

Required packages: `gazebo_ros`, `gazebo_ros2_control`, `ros2_controllers`,
`robot_state_publisher`, `xacro`, `launch_testing_ament_cmake`, and colcon.
The repository's existing Docker image does not declare the Gazebo dependencies.
Use separate build outputs to preserve the repository's existing build/install caches:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/lusi-assignment1/log build \
  --base-paths src/rover_sim_urc --packages-select rover_sim_urc \
  --build-base /tmp/lusi-assignment1/build --install-base /tmp/lusi-assignment1/install
source /tmp/lusi-assignment1/install/setup.bash
export ROS_DOMAIN_ID=77
export GAZEBO_MASTER_URI=http://127.0.0.1:11377
ros2 launch rover_sim_urc assignment1.launch.py
```

Use the same setup files and environment variables in another terminal:

```bash
ros2 run rover_sim_urc verify_assignment1.py
```

The verifier requires fresh joint feedback, tracks positive and negative wheel
velocities and steering angles, measures wheel displacement, and checks stopping.
Missing feedback, failed setpoint tracking, or a paused simulation fails within a
wall-clock deadline. It sends zero commands when exiting normally or on an exception.

To run the complete headless integration test (starts and stops its own Gazebo):

```bash
colcon --log-base /tmp/lusi-assignment1/log test \
  --build-base /tmp/lusi-assignment1/build --install-base /tmp/lusi-assignment1/install \
  --packages-select rover_sim_urc --event-handlers console_direct+
colcon test-result --test-result-base /tmp/lusi-assignment1/build --verbose
```

Stop any manual test simulation before running the integration test on the same
Gazebo master and ROS domain. For visualization, add `gui:=true` to the launch command
in an environment with a working display.

## Isolated Docker environment

From the repository root, with Docker Desktop WSL integration enabled:

```bash
docker build -t lusi-assignment1-sim src/rover_sim_urc
docker run --rm -e ROS_DOMAIN_ID=77 lusi-assignment1-sim bash -c \
  'source install/setup.bash && colcon test --packages-select rover_sim_urc --event-handlers console_direct+ && colcon test-result --verbose'
```

The image derives from the existing `urc_software_builder` image and builds only
this simulation package. It adds the missing `gazebo_ros` and `gazebo_ros2_control`
dependencies in a separate image layer. Building/testing it does not mount or write
the host repository's build caches, or update the existing rover image.

## ROS interfaces

| Topic | Type | Units / meaning |
|---|---|---|
| `/assignment1/wheel_velocity_controller/commands` | `std_msgs/msg/Float64MultiArray` | One value: front-left wheel rad/s |
| `/assignment1/steering_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | One value: front-left steering radians |
| `/assignment1/joint_state_broadcaster/joint_states` | `sensor_msgs/msg/JointState` | Measured wheel and steering positions/velocities |

For example, with only the original `assignment1.launch.py` running (these direct
commands bypass the gate and must not be used in the bridge/gate configuration):

```bash
ros2 topic pub --once /assignment1/wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray '{data: [2.0]}'
ros2 topic pub --once /assignment1/steering_position_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.3]}'
ros2 topic echo /assignment1/joint_state_broadcaster/joint_states
```

The wheel rotates about +Y; steering rotates about +Z. Steering limits are ±0.6 rad;
wheel velocity is configured for ±6 rad/s. Keep commands within those limits.
Controllers use ideal simulated velocity/position interfaces, without calibrated
motor dynamics or a command timeout. A last command can persist until replaced or
the simulator stops. This launch does not start any hardware nodes.

The existing `RoverDriveCommand2027` message and `DriveKinematics2027.hpp` are preserved.
Connecting that six-field command interface to a whole-rover simulation is a later
step; this milestone uses standard per-joint ROS controller interfaces.

Control configuration follows the upstream
[gazebo_ros2_control Humble documentation](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html)
and [forward command controller documentation](https://control.ros.org/humble/doc/ros2_controllers/forward_command_controller/doc/userdoc.html).

## Isolated 2027 message bridge and independent command gate

`assignment1_command_2027.launch.py` includes the original test-stand launch and adds
`command_bridge_2027.py` plus the separate `command_gate_2027.py` process.
The bridge accepts `cross_pkg_messages/msg/RoverDriveCommand2027`
on `/assignment1/rover_drive_command_2027`. Only `front_left_velocity` and
`front_left_steering` are supported. All other actuator fields must be zero;
unsupported, nonfinite, or fixture-out-of-range values stop and center the stand.
Valid messages must continue arriving within 0.5 s. The limits are those of this
test fixture, not hardware ratings. Neither component performs kinematic conversion.

The bridge publishes validated envelopes on
`/assignment1/validated_drive_command_2027`; it has no direct-controller publishers.
The gate independently validates all six fields and is the sole publisher to the
two controller-command topics in this launch configuration. Its monotonic receipt
lease expires after 0.5 s, checked by a steady-clock timer every 50 ms. It repeatedly
outputs zero wheel velocity and zero fixture steering at startup, after invalid
envelopes, and after expiration. Invalid envelopes clear the lease instead of
renewing it. Fresh valid envelopes restore motion. Repeating the cached controller
output does not renew the lease.

The bridge retains its original upstream timeout and produces valid zero envelopes
on invalid upstream requests or upstream silence. Gate validation is also tested
directly with the bridge suspended, so it does not depend on bridge sanitization.

**Receipt-time expiration is not replay protection.** An old, buffered, or repeatedly
replayed valid envelope is considered fresh when it arrives and can renew the gate
lease. Message header timestamps are not checked. This limitation is not resolved
by the depth-one input queue and no replay protection is claimed.

Build the existing message package into a separate overlay first, then enable the
additional integration tests. In an environment with the approved Gazebo dependencies:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/lusi-bridge-check/log build \
  --base-paths src/cross_pkg_messages_urc --packages-select cross_pkg_messages \
  --build-base /tmp/lusi-bridge-check/build --install-base /tmp/lusi-bridge-check/install
source /tmp/lusi-bridge-check/install/setup.bash
colcon --log-base /tmp/lusi-bridge-check/log build \
  --base-paths src/rover_sim_urc --packages-select rover_sim_urc \
  --build-base /tmp/lusi-bridge-check/build --install-base /tmp/lusi-bridge-check/install \
  --cmake-args -DROVER_SIM_TEST_2027_COMMAND_BRIDGE=ON
source /tmp/lusi-bridge-check/install/setup.bash
export ROS_DOMAIN_ID=79
export GAZEBO_MASTER_URI=http://127.0.0.1:11379
colcon --log-base /tmp/lusi-bridge-check/log test \
  --base-paths src/rover_sim_urc --packages-select rover_sim_urc \
  --build-base /tmp/lusi-bridge-check/build --install-base /tmp/lusi-bridge-check/install \
  --event-handlers console_direct+ --ctest-args -j1
colcon test-result --test-result-base /tmp/lusi-bridge-check/build --verbose
```

The standalone Docker recipe above still runs the original test only. To test the
bridge with Docker, use the existing approved `lusi-assignment1-sim` image with both
package directories mounted read-only and a separate writable overlay directory.
The latest executed commands are in `/tmp/lusi-assignment1-command-gate-r2/run.sh` and
the test evidence is recorded in `docs/ASSIGNMENT1_VERIFICATION.md`.

For manual testing after building/sourcing that overlay, launch only the bridge
configuration (do not also launch the original stand):

```bash
ros2 launch rover_sim_urc assignment1_command_2027.launch.py
# In a second terminal with the same overlay/domain:
ros2 topic pub --rate 10 /assignment1/rover_drive_command_2027 \
  cross_pkg_messages/msg/RoverDriveCommand2027 \
  '{front_left_velocity: 2.0, front_left_steering: 0.3}'
```

Stopping that publisher triggers the bridge's tested stop-and-center behavior.
Do not use this adapter for the full four-wheel rover: that model has no steering
joints, and its dimensions differ from the provisional 2027 kinematics values.

### Bridge process failure containment

The bridge/gate launch requests a launch-wide shutdown whenever either process
exits, including a crash or SIGKILL. A suspended bridge is handled by the gate's
independent timeout while Gazebo continues running. Run this configuration as
an isolated launch: including it in a larger launch also applies that shutdown to
the enclosing launch session. The original direct-controller launch is unchanged.

With `ROVER_SIM_TEST_2027_COMMAND_BRIDGE=ON`, the suite now runs five integration
tests. `test/test_command_bridge_failure_launch.py` verifies real wheel and
steering motion, kills only its own supervised bridge process, and requires a clean
Gazebo shutdown within five seconds. It observes the production launch in a child
LaunchService so the expected shutdown does not terminate the test runner itself.

`test/test_command_gate_launch.py` verifies startup without upstream commands,
sole controller-output ownership, bridge SIGSTOP expiration, sustained stopping and
repeated zero outputs, ten independently rejected gate inputs, continued invalid
input, gate-input silence, recovery on fresh upstream commands, and gate SIGKILL
containment. Existing motion, validation, timeout, and bridge-exit assertions remain.
`test/test_command_gate_startup_launch.py` separately starts the gate and fixture
without a bridge or any envelope publisher, and verifies repeated zero controller
commands with fresh stopped joint feedback.

This terminates the simulation; it does not establish zero joint velocity after
failure or a physical motor stop. A hung gate, supervisor failure, and competing
direct command publishers remain outside this protection. Normal process shutdown
only attempts a final zero publication; delivery is not guaranteed. These software
processes are not a hardware safety mechanism, and test timing is not a hard
real-time stop guarantee. Paused-world behavior and replay protection are unverified.
