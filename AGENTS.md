# AGENTS.md

Instructions for AI coding agents (Claude Code, Codex, Copilot, Cursor, and others) working in `urc_software`.
Humans: this is a short rulebook, not documentation. The docs it points to are the source of truth.

Last verified against the code: 2026-11-22

## What this repo is

ROS 2 Humble software (mostly C++; the Science module and voice commands are Python) for the Lehigh University Space Initiative (LUSI) Mars rover, competing in the University Rover Challenge (URC).
Everything builds and ships as one Docker image; the container's mode argument (`run_nodes.sh`) picks which nodes start on which computer.

## Read only what your task needs

Do not read every doc up front. Jump to the section for your task:

| Task | Read |
|------|------|
| Which node runs where, data flow | `README.md` "System architecture" |
| Adding, renaming, or debugging a topic | `README.md` "Topic map" (source of truth for every topic) |
| Changing a custom message | `README.md` "Message interfaces", then the `.msg` header in `src/cross_pkg_messages_urc/msg/` |
| Motor, CAN, or PID code | `README.md` "Shared code", then `docs/DRIVETRAIN.md` |
| Build, run modes, deploy | `README.md` "Building" and "Running"; `QUICKSTART.md` for the fast loop |
| Branching, commits, comment style | `CONTRIBUTING.md` |
| Unfamiliar C++ or ROS 2 construct | `docs/CPP_ROS2_PRIMER.md` |
| Something looks broken or half-wired | `README.md` "Known integration issues" and "Housekeeping notes"; it may already be documented |

Every first-party source file has a header listing which computer runs it, its launch file, and the topics it publishes and subscribes to.
Read that header before the rest of the file.

## Package map

| Folder | Runs on | What it does |
|--------|---------|--------------|
| `src/base_station_urc` | Operator laptop | ImGui GUI (`src/gui/panels`), joystick and SpaceMouse mappers, video receiver |
| `src/main_computer_urc` | Rover main computer | MoveIt arm stack, `DriveTrainManager`, `VideoStreamer`, `MockArmHardware` (the real arm bridge, despite the name) |
| `src/driveline_urc` | Driveline Pi | Wheel motors (SPARK MAX over CAN) |
| `src/arm_urc` | Arm Pi | Arm joint motors (SPARK MAX over CAN) |
| `src/moveit_config_urc` | Main computer | MoveIt Setup Assistant output; config only |
| `src/navigation_urc` | Main computer / sim | GNSS waypoint follower and `fake_gps_node.py` |
| `src/drivetrain_2027_urc` | Main computer (future) | 2027 drivetrain mode arbitration and kinematics; mock only, no hardware output |
| `src/rover_sim_urc` | Dev machine | Gazebo sim of the 2027 rover; not in the Docker build (needs Gazebo) |
| `src/voice_commands_urc` | Main computer (presumably; not launched yet) | Python voice-command state machine (stay/come/follow/fetch) driving `cmd_vel` |
| `src/science_urc` | Science module | Python nodes on `/science/*` topics; drivers in `science_urc/hardware/` |
| `src/cross_pkg_messages_urc` | All | Custom messages; ROS package name is `cross_pkg_messages` |
| `src/shared_code` | Arm and driveline Pis | `CANDriver`, `SparkMax`, `MotorManager`, `pid` |

## Build and verify

There is no local ROS 2 install; everything runs in Docker.

```bash
git submodule update --init --recursive
./softwareUpdate/dockerBuild.sh          # full build; must print "Code compiled successfully!"
```

Fast loop for one package (dev shell, repo mounted):

```bash
docker run --rm -it --net=host -v "$(pwd):/ros2_ws" -w /ros2_ws urc_software_builder bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --merge-install --packages-select <package>
```

Hardware-free checks, in order of cost:

1. `nav_sim` mode: `./src/navigation_urc/launch/launchScript.sh` (navigation changes)
2. `hootl` mode: base station + main computer + driveline on one machine, no hardware (anything on the drive or GUI path)

Expect one-time `CAN bus N (canN) unavailable` warnings off the rover; they are normal.

Before declaring a task done:

- The full Docker build passes (CI runs the same script on every PR)
- The relevant hardware-free mode runs without new errors
- `clang-format` has been applied to changed C++ files (command in `CONTRIBUTING.md`)
- New or changed files follow the comment style in `CONTRIBUTING.md` (file header, function header blocks, `// Syntax:` and `// System:` tags)

If you cannot run Docker in your environment, say so plainly.
Never claim code builds or runs unless you actually built or ran it.

## Git rules

- Never commit or push to `main`. Work on a branch from a freshly pulled `main` and open a PR.
- Never force-push a branch someone else is using.
- Conventional Commits: `feat:`, `fix:`, `docs:`, `refactor:`, `test:`, `build:`, `ci:`, `chore:`.
- One focused change per PR. Fill in the PR template; the software lead reviews every PR.
- Do not keep committing on a branch after its PR is squash-merged.

## Hardware safety (hard rules)

This code moves a ~40 kg rover and a robotic arm. Code that runs on the rover can break hardware or hurt someone.

- Before changing any of the following, stop, ask the human, and explain what you want to change and why:
  - CAN IDs, CAN bus numbers, or motor direction signs
  - Gear ratios, current limits, PID gains, speed scaling, or joint limits
  - Loss-of-signal timeouts and heartbeat logic in `src/shared_code/MotorManager*`
- If the human approves, call the change out in the PR description.
- Never remove or weaken a safety stop, timeout, or limit to make something "work".
- A change is not verified on hardware just because it builds or runs in `hootl`. Say so in the PR.
- Never run `softwareUpdate/urc_deploy.py` or connect to rover hosts (`10.0.0.x`); deploying restarts software on the physical rover. That script also holds credentials: never copy, print, or move them.

## Do not edit

- `libs/` (git submodules) and `src/slidar_ros2/` (LiDAR driver submodule)
- Any `include/cs_libguarded/` tree (vendored)
- `src/shared_code/pid.{h,cpp}` (vendored MIT code)
- `build/`, `install/`, `log/` (generated)

Do not search or read these either unless the task is specifically about them; they are most of the repo by size.

## Things that will trip you up

- **The code can lag the physical rover.** Before changing code that depends on physical values (wheel count, wheel radius, track width, gear ratios, steering, CAN map), compare the code with `docs/DRIVETRAIN.md` and ask the human whether the physical design has changed since. Do not "fix" a mismatch in passing; flag it.
  - Current example: the rover has 4 driven wheels with 2 steered front wheels, but `main` still drives 6 wheels with tank (skid) steering.
- **Two turn conventions.** `JoyMapper` and `DriveTrainManager` steer with `angular.y` in deg/s; `WaypointFollower` uses the ROS standard `angular.z` in rad/s. See "Known integration issues" in `README.md` before touching `cmd_vel`.
- **Topics and messages are contracts across computers.** When you rename a topic or change a message field, unit, or axis convention, update every publisher and subscriber in the same change, plus the README topic map. Search for the topic string both with and without a leading `/`; the code uses both.
- **Package name mismatch.** Folder `src/cross_pkg_messages_urc/` builds package `cross_pkg_messages`.
- **CMake globbing.** Sources are collected with `file(GLOB)`; after adding or removing files, rebuild clean or CMake will not see them.
- **CI only proves `science_urc` installs.** It is in the Docker build now, but it is pure Python with no launch mode, so a passing build does not run or lint the Science code. Run `colcon test --packages-select science_urc` in the dev shell.

## Keeping docs true

- Comments state only what the code guarantees. If you are inferring intent, write `presumably` or `VERIFY`, never fact.
- Delete dead code instead of commenting it out.
- Anyone changing code keeps this file current. If your change makes a statement in `AGENTS.md`, `README.md`, `CONTRIBUTING.md`, or `docs/DRIVETRAIN.md` wrong, update that document in the same PR.
- When you edit this file, update the "Last verified" date at the top. Keep it under about 150 lines: move detail into the README and leave a pointer here.
