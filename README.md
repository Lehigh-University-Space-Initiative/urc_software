# URC Software

Software for the LUSI (Laurentian University) rover competing in the University Rover
Challenge (URC). The rover is driven from a **base station** laptop over the network; on
the rover, a **main computer** coordinates the arm and cameras while dedicated **Raspberry
Pis** drive the wheels and the arm hardware.

Everything is written as a set of [ROS 2 Humble](https://docs.ros.org/en/humble/) packages
(mostly C++) and is built and shipped as a single Docker image, so the same image runs on
the base station and on every on-rover computer — the command-line argument passed to the
container selects which nodes start.

---

## Table of contents

- [System architecture](#system-architecture)
- [Repository layout](#repository-layout)
- [Prerequisites](#prerequisites)
- [Setup](#setup)
- [Building](#building)
- [Running](#running)
- [Deploying to the rover](#deploying-to-the-rover)
- [Package reference](#package-reference)
- [Message interfaces](#message-interfaces)
- [Shared code](#shared-code)
- [External libraries (submodules)](#external-libraries-submodules)
- [Documentation](#documentation)
- [Continuous integration](#continuous-integration)
- [Housekeeping notes](#housekeeping-notes)

---

## System architecture

The system spans four computers. All of them run the same `urc_software` Docker image; the
entrypoint argument (`base_station`, `main_computer`, `driveline`, `arm`, …) decides which
ROS 2 launch file runs. See [`run_nodes.sh`](run_nodes.sh).

| Computer | Runs | Package | Responsibility |
|----------|------|---------|----------------|
| **Base station** (operator laptop) | `base_station` | `base_station_urc` | Operator GUI, joystick/SpaceMouse input, video display |
| **Main computer** (rover, `10.0.0.10`) | `main_computer` | `main_computer_urc` + `moveit_config_urc` | Arm motion planning (MoveIt), camera streaming, drive-command relay |
| **Driveline Pi** (rover, `10.0.0.20`) | `driveline` | `driveline_urc` | Wheel motor control over CAN |
| **Arm Pi** (rover) | `arm` | `arm_urc` | Arm joint motor control over CAN |

Nodes communicate over ROS 2 topics carrying the custom messages defined in
`cross_pkg_messages`. The high-level flow:

```
 Joystick / SpaceMouse
          │
          ▼
 ┌──────────────────┐   /roverDriveCommands (RoverComputerDriveCMD)
 │  base_station_urc│──────────────────────────────────────────────┐
 │  - JoyMapper     │   /armInputRaw (ArmInputRaw)                   │
 │  - SpaceMouse    │─────────────────────────────┐                 │
 │  - GroundStation │                              │                 │
 │    GUI           │◀── /video_stream/compressed  │                 │
 └──────────────────┘        (from rover cameras)  │                 │
                                                    ▼                 ▼
                                       ┌─────────────────────┐  ┌───────────────┐
                                       │   main_computer_urc │  │ driveline_urc │
                                       │  - MoveIt / Servo   │  │  - MotorCtr   │
                                       │  - DriveTrainManager│──│    (CAN →     │
                                       │  - VideoStreamer    │  │  SparkMax)    │
                                       └─────────┬───────────┘  └───────────────┘
                                                 │ /roverArmCommands
                                                 ▼
                                          ┌───────────────┐
                                          │    arm_urc    │
                                          │ - ArmMotorMgr │
                                          │  (CAN + GPIO) │
                                          └───────────────┘
```

Motor control on the rover uses **REV SparkMax** controllers on a CAN bus (via a Waveshare
2-channel CAN HAT on the Raspberry Pis); the CAN and PID logic lives in
[`src/shared_code`](src/shared_code).

---

## Repository layout

```
urc_software/
├── Dockerfile                 # Multi-stage build of the urc_software image (ROS 2 Humble)
├── run_nodes.sh               # Container entrypoint; arg selects which nodes launch
├── Doxyfile                   # Doxygen configuration (API docs)
├── softwareUpdate/
│   ├── dockerBuild.sh         # Builds the image + compiles the ROS workspace inside it
│   └── urc_deploy.py          # rsync + remote Docker build + systemd restart on the rover
├── src/
│   ├── base_station_urc/      # Operator-side GUI, input mapping, video receiver
│   ├── main_computer_urc/     # Arm MoveIt stack, drive relay, camera streaming
│   ├── driveline_urc/         # Wheel motor controller (Raspberry Pi)
│   ├── arm_urc/               # Arm motor controller (Raspberry Pi)
│   ├── moveit_config_urc/     # MoveIt Setup Assistant output for the 2-DOF arm
│   ├── cross_pkg_messages_urc/# Custom ROS 2 message definitions (package: cross_pkg_messages)
│   ├── shared_code/           # CAN driver, SparkMax, PID, MotorManager (shared by rover pkgs)
│   └── slidar_ros2/           # Slamtec LiDAR driver (git submodule)
├── libs/                      # Third-party libraries (git submodules) — see below
└── urcAssets/                 # Runtime image assets (e.g. GUI "no downlink" placeholder)
```

> **Note:** the folder `src/cross_pkg_messages_urc/` defines a package whose ROS name is
> `cross_pkg_messages` (see its `CMakeLists.txt`/`package.xml`). Refer to it as
> `cross_pkg_messages` in launch files and dependencies.

---

## Prerequisites

- **Docker** (the workspace is built and run entirely inside containers; you do not need a
  local ROS 2 install).
- **git** with submodule support.
- An **X11** display server if you want to see the GUI or RViz (Linux desktop, or an X
  server on Windows/macOS).
- Tested on **Ubuntu 22.04**, but any Linux distro with Docker should work.

> **On Windows?** Build and run through **WSL 2 + Docker** — see
> [WSL_SETUP.md](WSL_SETUP.md). WSL can compile the workspace and run the
> GUI/simulation, but cannot exercise real rover hardware (CAN, GPIO, USB).

---

## Setup

1. Clone the repository, then initialize the submodules from the repo root:

   ```bash
   git submodule init
   git submodule update
   ```

   This pulls in the external libraries: the GUI (**ImGui**), Raspberry Pi GPIO access
   (**pigpio**), sockets (**sockpp**), the LiDAR driver (**sllidar_ros2**), and the Doxygen
   HTML theme (**doxygen-awesome-css**). See
   [External libraries](#external-libraries-submodules).

2. Build the Docker image (see below).

---

## Building

The build runs entirely in Docker. From the repo root:

```bash
./softwareUpdate/dockerBuild.sh
```

This script:

1. Builds the `urc_software_builder` image (the `Dockerfile`'s build stage, with all ROS 2 /
   OpenGL / OpenCV / MoveIt build dependencies).
2. Runs `colcon build --merge-install` inside that image, mounting the repo so the `build/`
   directory is cached on the host between builds. Packages are built in two passes so the
   generated messages are available first:
   - `cross_pkg_messages`
   - then `base_station_urc`, `main_computer_urc`, `driveline_urc`, `sllidar_ros2`,
     `moveit_config_urc`, `arm_urc`
3. Builds the final runtime image, tagged both `urc_software` and
   `10.0.0.10:65000/urc_software` (the rover's local registry).

On success it prints **"Code compiled successfully!"**; a compile failure exits non-zero
with **"Code did not compile!"**.

---

## Running

Each computer is started by running the container with the appropriate mode argument.
`run_nodes.sh` (the image entrypoint) sources the workspace and dispatches on that argument:

| Mode | Launches |
|------|----------|
| `base_station` | `base_station_urc` (GUI + input mappers + video receiver) |
| `main_computer` | `main_computer_urc` (MoveIt/servo, drive relay, video streamer) |
| `driveline` | `driveline_urc` (wheel motor controller) |
| `arm` | `arm_urc` (arm motor controller) |
| `rviz` | `main_computer_urc` RViz-only visualization launch |
| `hootl` | Hardware-out-of-the-loop: runs base station **+** main computer **+** driveline together on one machine for local testing |
| `manual` | Drops into an interactive `bash` shell in the container (run with `-it`) |

Convenience wrappers under each package's `launch/launchScript.sh` run the correct
`docker run` command (device passthrough, host networking, `DISPLAY`, etc.):

```bash
# Base station (operator laptop)
./src/base_station_urc/launch/launchScript.sh

# Rover main computer
./src/main_computer_urc/launch/launchScript.sh

# Driveline / arm Raspberry Pis
./src/driveline_urc/launch/launchScript.sh
./src/arm_urc/launch/launchScript.sh
```

### X11 / GUI on Linux

Once per boot, allow the container to draw on your display:

```bash
xhost +
```

Then launch the base station as above. The GUI (ImGui + GLFW) and RViz both need this.

---

## Deploying to the rover

`softwareUpdate/urc_deploy.py` automates pushing code to the rover, rebuilding, and
restarting the on-rover software. It uses SSH/`rsync` to the main computer (`10.0.0.10`) and
the driveline Pi (`10.0.0.20`), and restarts the `lusi-software.service` systemd unit on
each.

```bash
# Full deploy (default when no flags are given): sync + remote Docker build
python3 ./softwareUpdate/urc_deploy.py

# Individual stages
python3 ./softwareUpdate/urc_deploy.py --rsync    # sync files only
python3 ./softwareUpdate/urc_deploy.py --docker   # sync + remote Docker build + registry push
python3 ./softwareUpdate/urc_deploy.py --deploy   # sync + build + restart the systemd service
python3 ./softwareUpdate/urc_deploy.py --help
```

Requires Python with `paramiko` installed on the machine running the deploy. Host IPs and
credentials are defined at the top of the script.

---

## Package reference

### `base_station_urc`
Runs on the operator's laptop. Executables (see its `CMakeLists.txt`):

- **`GroundStationGUI`** — the ImGui/GLFW operator interface (`src/gui`). Composed of
  panels: COMS status, telemetry, system control, and video view. Displays the rover camera
  feed and rover state.
- **`JoyMapper_node`** — reads two `joy` topics (`/joy0`, `/joy1`) and maps controller axes
  to drive/arm command topics.
- **`SpaceMouseMapper_node`** — maps a 3D SpaceMouse into arm end-effector input
  (`/armInputRaw`).
- **`LUSIVisionStreamer_node`** — telemetry/video streaming helper built on `sockpp`.
- **`ArmCommandEncoder_node`** — MoveIt Servo–based arm command encoder (built and
  installed; **not currently wired into `base_station_launch.py`** — see
  [Housekeeping notes](#housekeeping-notes)).

Launch: `launch/base_station_launch.py` also starts two `joy` driver nodes and an
`image_transport republish` node to decompress the incoming video stream.

### `main_computer_urc`
Runs on the rover's main computer. Executables:

- **`DriveTrainManager_node`** — converts high-level `Twist` drive input into per-side wheel
  velocities and republishes `RoverComputerDriveCMD` for the driveline Pi.
- **`VideoStreamer_node`** — captures camera frames (OpenCV) and publishes them over
  `image_transport`.
- **`StatusLED_node`** — rover status-light controller (built and installed; **currently a
  stub and commented out of the launch file** — see [Housekeeping notes](#housekeeping-notes)).
- **`MockArmHardware`** (`mock_arm_hw`) — a `ros2_control` `SystemInterface` plugin
  (`plugin.xml`) used as the arm's hardware interface in the URDF, exercised via the MoveIt
  stack. This plugin **is** referenced by `description/robot.ros2_control.xacro`.

Launch: `launch/main_computer_launch.py` brings up the full MoveIt stack (move_group,
controller manager, joint-state broadcaster, arm controller, MoveIt Servo), RViz, the video
pipeline, and the drive manager. `launch/rviz_gui_launch.py` is the visualization-only
variant. `description/` holds the robot URDF/xacro, ros2_control, and RViz config; `config/`
holds the servo simulation config.

### `driveline_urc`
Runs on the driveline Raspberry Pi. Single executable **`MotorCtr_node`**, which subscribes
to `/roverDriveCommands` and drives six wheel motors (three per side) as **SparkMax**
controllers over CAN, using the shared `MotorManager`/`CANDriver` code. Links `pigpio` for
Pi hardware access.

### `arm_urc`
Runs on the arm Raspberry Pi. Single executable **`ArmMotorManager`**, which subscribes to
`/roverArmCommands` and `/armInputRaw`, drives the arm joint motors over CAN (shared
`MotorManager`), and publishes measured joint positions on `/roverArmPos`. Links `pigpio`.

### `moveit_config_urc`
MoveIt Setup Assistant output for the 2-DOF arm: SRDF, kinematics, joint limits, controller
configs, RViz config, and the `demo`/`move_group`/`moveit_rviz`/`spawn_controllers`/etc.
launch files. Consumed by `main_computer_urc`'s launch file. Not a code package — it just
installs config and launch files.

### `cross_pkg_messages` (folder `cross_pkg_messages_urc`)
Defines the custom ROS 2 messages shared across packages. See
[Message interfaces](#message-interfaces).

---

## Message interfaces

Defined in `src/cross_pkg_messages_urc/msg`:

| Message | Purpose | Key fields |
|---------|---------|-----------|
| **`RoverComputerDriveCMD`** | Wheel commands base→rover | `cmd_l` (L front/center/back), `cmd_r` (R front/center/back), normalized ±1 |
| **`RoverComputerArmCMD`** | Arm joint commands to the arm Pi | `cmd_b` (base), `cmd_s` (shoulder), `cmd_e` (elbow), `cmd_w` (wrist roll/pitch/yaw) |
| **`ArmInputRaw`** | Raw operator arm input (e.g. SpaceMouse) | `linear_input`, `angular_input`, `left_btn`, `right_btn` |
| **`GPSData`** | GPS fix | `status`, `lla`, `speed`, `course`, `sats`, `lla_acc` |

---

## Shared code

`src/shared_code` is compiled into the rover-side packages (arm and driveline) via
`file(GLOB ...)` in their `CMakeLists.txt`:

- **`CANDriver` / `SparkMax`** (`CANDriver.h/.cpp`) — CAN bus communication for REV SparkMax
  motor controllers (designed for the Waveshare 2-channel Raspberry Pi CAN HAT).
- **`MotorManager`** (`MotorManager.h/.cpp`) — abstract base managing a set of motors,
  heartbeats, loss-of-signal timeout, and read/write ticks. Subclassed by
  `DriveTrainMotorManager` (driveline) and `ArmMotorManager` (arm).
- **`pid`** (`pid.h/.cpp`) — PID controller used for closed-loop motor control.
- **`Limits.h`**, **`Logger.h`** — helpers.

---

## External libraries (submodules)

Declared in `.gitmodules`:

| Submodule | Path | Used for |
|-----------|------|----------|
| **ImGui** (docking branch) | `libs/imgui` | Base-station GUI |
| **pigpio** | `libs/pigpio` | Raspberry Pi GPIO access (arm/driveline) |
| **sockpp** | `libs/sockpp` | Sockets for the vision streamer |
| **doxygen-awesome-css** | `libs/doxygen-awesome-css` | Doxygen HTML theme |
| **sllidar_ros2** | `src/slidar_ros2` | Slamtec LiDAR ROS 2 driver |

---

## Documentation

API documentation is generated with **Doxygen** using [`Doxyfile`](Doxyfile) (styled with
doxygen-awesome-css). It excludes the LiDAR submodule, `moveit_config_urc`, and the
`cs_libguarded` vendored headers. To generate locally:

```bash
doxygen Doxyfile      # output written to docs/html/
```

The `Deploy Doxygen to Github Pages` workflow publishes these docs to GitHub Pages on every
push to `main`.

---

## Continuous integration

Two GitHub Actions workflows (`.github/workflows/`):

- **`docker-image-ci.yml`** — on pushes to `main` and all pull requests, checks out
  submodules and runs `./softwareUpdate/dockerBuild.sh` to verify the whole workspace still
  compiles.
- **`github_pages_doxygen_deploy.yaml`** — builds and deploys the Doxygen docs to GitHub
  Pages on pushes to `main`.

---

## Contributing

Code style, comment conventions, and file-structure guidelines live in
[CONTRIBUTING.md](CONTRIBUTING.md). Formatting is enforced by
[`.clang-format`](.clang-format) and [`.editorconfig`](.editorconfig).

## Housekeeping notes

During documentation the following **dead/outdated items were removed** from the tree:

- `src/arm_urc/plugin.xml` and `src/driveline_urc/plugin.xml` — `ros2_control` plugin
  descriptors for `arm_urc/ArmHardware` and `driveline_urc/DrivelineHardware`. Neither class
  exists in the source, neither library was built or installed, and no URDF/xacro referenced
  them. (The **only** live hardware plugin is `main_computer_urc/MockArmHardware`, which is
  kept.)
- `src/driveline_urc/src/StepperDriver.{h,cpp}` — an older stepper-motor driver, never
  instantiated. The driveline now drives SparkMax motors over CAN via `MotorManager`.
- Stale build references in `src/arm_urc/CMakeLists.txt` (a dead `ARM_URC_HW_SRC` variable
  pointing at a non-existent `src/ArmHardware.cpp`, plus commented-out `plugin.xml` install).
- The `ground_input` case in `run_nodes.sh`, which launched a non-existent
  `ground_input_urc` package.

The following are **built but not currently active** and were left in place (they are
coherent, potentially in-progress features rather than clearly abandoned code) — revisit if
you want to trim further:

- **`ArmCommandEncoder_node`** (`base_station_urc`) — built and installed, but not included
  in any launch file.
- **`StatusLED_node`** (`main_computer_urc`) — built and installed, but currently a stub and
  commented out of `main_computer_launch.py`.
- **`SoftwareDebugPanel`** (`base_station_urc` GUI) — compiled and `#include`d in
  `guiMain.cpp`, but not added to the active panel list.
