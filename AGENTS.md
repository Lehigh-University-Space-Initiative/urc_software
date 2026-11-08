# AGENTS.md

Instructions for AI coding agents (Claude Code, Codex, Copilot, Cursor, and others) working in `urc_software`.
Humans: this is a short rulebook, not documentation. The docs it points to are the source of truth.

## What this repo is

ROS 2 Humble software (mostly C++, Science module in Python) for the Lehigh University Space Initiative (LUSI) Mars rover, competing in the University Rover Challenge (URC).
Everything builds and ships as one Docker image; the container's mode argument (`run_nodes.sh`) picks which nodes start on which computer.

Read before changing anything non-trivial:

- `README.md`: architecture, topic map, run modes, package reference, known integration issues
- `CONTRIBUTING.md`: branching, commits, formatting, and the required comment style
- `QUICKSTART.md`: the hardware-free edit-build-run loop
- `docs/DRIVETRAIN.md`: physical rover values (geometry, motors, CAN IDs, gearing); estimates, see its header
- `docs/CPP_ROS2_PRIMER.md`: C++ and ROS 2 constructs the comments refer to

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

## Do not edit

- `libs/` (git submodules) and `src/slidar_ros2/` (LiDAR driver submodule)
- Any `include/cs_libguarded/` tree (vendored)
- `src/shared_code/pid.{h,cpp}` (vendored MIT code)
- `build/`, `install/`, `log/` (generated)

## Things that will trip you up

- **The code can lag the physical rover.** Before changing code that depends on physical values (wheel count, wheel radius, track width, gear ratios, steering, CAN map), compare the code with `docs/DRIVETRAIN.md` and ask the human whether the physical design has changed since. Do not "fix" a mismatch in passing; flag it.
  - Current example: the rover has 4 driven wheels with 2 steered front wheels, but `main` still drives 6 wheels with tank (skid) steering.
- **Two turn conventions.** `JoyMapper` and `DriveTrainManager` steer with `angular.y` in deg/s; `WaypointFollower` uses the ROS standard `angular.z` in rad/s. See "Known integration issues" in `README.md` before touching `cmd_vel`.
- **Package name mismatch.** Folder `src/cross_pkg_messages_urc/` builds package `cross_pkg_messages`.
- **CMake globbing.** Sources are collected with `file(GLOB)`; after adding or removing files, rebuild clean or CMake will not see them.
- **`science_urc` is not in the Docker build.** `dockerBuild.sh` does not build it and no launch mode starts it, so a passing CI build says nothing about Science code.

## Keeping docs true

- Comments state only what the code guarantees. If you are inferring intent, write `presumably` or `VERIFY`, never fact.
- Delete dead code instead of commenting it out.
- Anyone changing code keeps this file current. If your change makes a statement in `AGENTS.md`, `README.md`, `CONTRIBUTING.md`, or `docs/DRIVETRAIN.md` wrong, update that document in the same PR.
