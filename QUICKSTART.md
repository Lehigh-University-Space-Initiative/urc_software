# Quickstart: run the rover software on your laptop (no rover needed)

This gets you from a fresh clone to **watching the software work and changing it**, without
any rover hardware. Budget about an hour the first time, mostly waiting on the first build.

| Step | What you get | Needs |
|------|--------------|-------|
| [1. Install the tools](#1-install-the-tools) | Docker + git | One-time setup |
| [2. Get the code and build it](#2-get-the-code-and-build-it) | The `urc_software` Docker image | ~30–60 min the first time |
| [3. Run the navigation simulation](#3-run-the-navigation-simulation) | A simulated rover driving to a GPS waypoint | Nothing extra (no display, no devices) |
| [4. The fast edit-build-run loop](#4-the-fast-edit-build-run-loop) | Change code and see the result in about a minute | — |
| [5. Run the operator GUI and full simulation](#5-run-the-operator-gui-and-full-simulation) | The ground station window, and `hootl` mode | A display (WSLg on Windows) |
| [6. Where to go next](#6-where-to-go-next) | Good first tasks and deeper docs | — |

---

## 1. Install the tools

- **Linux (Ubuntu 22.04 tested):** install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) and git
- **Windows:** follow [docs/WSL_SETUP.md](docs/WSL_SETUP.md) (WSL 2 + Docker Desktop), then do everything below inside the Ubuntu terminal
- **macOS:** install Docker Desktop; steps 2–4 work, and the GUI needs an X server such as XQuartz

Check Docker works before continuing:

```bash
docker run --rm hello-world
```

## 2. Get the code and build it

```bash
git clone https://github.com/Lehigh-University-Space-Initiative/urc_software.git
cd urc_software
git submodule update --init --recursive     # pulls ImGui, pigpio, sockpp, the LiDAR driver
./softwareUpdate/dockerBuild.sh
```

The first build downloads ROS 2, MoveIt, and OpenCV (several GB), then compiles every package.
It ends with **"Code compiled successfully!"**. If it prints "Code did not compile!" instead,
scroll up to the first `error:` line, or see [Troubleshooting](#troubleshooting).

What just happened: `dockerBuild.sh` built a "builder" image with every dependency, ran
`colcon build` inside it with this folder mounted (so `build/` and `install/` appear here on your
machine), then packaged `install/` into the runnable `urc_software` image.

## 3. Run the navigation simulation

```bash
./src/navigation_urc/launch/launchScript.sh
```

This starts two nodes that talk to each other over ROS 2 topics:

```
 FakeGPS (simulated rover)  --/gps_data-->  WaypointFollower
          ^                                        |
          +------------------/cmd_vel--------------+
```

- `WaypointFollower` reads the rover's position, works out the distance and compass bearing to a
  target waypoint, and publishes a drive command on `/cmd_vel`
- `FakeGPS` pretends to be the rover: it moves according to `/cmd_vel` and publishes the new position

You should see the rover turn toward the target, then the distance shrink each second until it
arrives (about 30 seconds). Real output, trimmed:

```
[FakeGPS]: FakeGPS running, starting at (38.4061, -110.7918)
[WaypointFollower]: distance=21.19m bearing=38.1 heading=0.0 cmd(lin=0.35 ang=-0.80)
[WaypointFollower]: distance=20.82m bearing=38.4 heading=32.5 cmd(lin=0.56 ang=-0.18)
[WaypointFollower]: distance=20.18m bearing=38.4 heading=38.0 cmd(lin=0.60 ang=-0.01)
[WaypointFollower]: distance=19.58m bearing=38.4 heading=38.4 cmd(lin=0.60 ang=-0.00)
...
[WaypointFollower]: distance=4.22m bearing=38.4 heading=38.4 cmd(lin=0.60 ang=-0.00)
[WaypointFollower]: distance=3.62m bearing=38.4 heading=38.4 cmd(lin=0.60 ang=-0.00)
[WaypointFollower]: Arrived at target (2.96 m away)
```

Reading it: `bearing` is the compass direction to the target, `heading` is the direction the
rover is moving, and `cmd` is the command sent (`lin` forward speed in m/s, `ang` turn rate in
rad/s). The rover first turns until `heading` matches `bearing` (negative `ang` = turning right),
then drives straight at full speed (0.6 m/s) until it's within 3 m.

Press **Ctrl+C** to stop. The start point is the Mars Desert Research Station in Utah (where URC is
held), and the default target is about 21 m to the northeast.

Try a different target without changing any code:

```bash
docker run --rm -it --net=host --entrypoint bash urc_software -c \
  "source /ros2_ws/install/setup.bash && ros2 launch navigation_urc navigation_sim_launch.py target_lat:=38.4070 target_lon:=-110.7910"
```

(`--entrypoint bash` replaces the image's usual startup script, `run_nodes.sh`, so you can run any ROS 2 command.)

## 4. The fast edit-build-run loop

Rebuilding the whole image for every change is slow. Instead, open a **dev shell**: a shell inside
the builder image with this folder mounted, where you build and run just the package you're editing.

```bash
# From the repo root; leave this shell open while you work
docker run --rm -it --net=host -v "$(pwd):/ros2_ws" -w /ros2_ws urc_software_builder bash
```

Inside the dev shell:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash                                 # this repo's already-built packages

colcon build --merge-install --packages-select navigation_urc    # rebuild only what you changed
source install/setup.bash                                 # pick up the new build
ros2 launch navigation_urc navigation_sim_launch.py
```

Edit files in your normal editor on your machine; they're the same files the dev shell sees. A
single-package rebuild takes about a minute. Run `./softwareUpdate/dockerBuild.sh` again only when
you want the final `urc_software` image updated (e.g. before deploying, or to use the `launchScript.sh` wrappers).

**A first change to try:** in [`src/navigation_urc/src/WaypointFollower/main.cpp`](src/navigation_urc/src/WaypointFollower/main.cpp),
find `computeDriveCommand` and change `kAngular` from `0.03` to `0.1`. Rebuild and rerun as above,
and watch how the heading swings past the target before settling (the comment by that log line
explains what oscillation means). Change it back afterwards.

Handy tools in a second dev shell while the sim runs (open another terminal and start another dev shell):

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 topic list                       # every topic in the running system
ros2 topic echo /cmd_vel              # watch the drive commands live
ros2 param set /WaypointFollower target_lat 38.4065   # move the waypoint while it's running
```

## 5. Run the operator GUI and full simulation

These need a display. On Linux run `xhost +local:` once per login; on Windows, WSLg already provides one.

```bash
# Just the ground station GUI
./src/base_station_urc/launch/launchScript.sh

# Hardware-out-of-the-loop: base station + main computer + driveline on one machine
docker run --rm -it --net=host --ipc=host --pid=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix urc_software hootl
```

In `hootl` mode the GUI's COMS panel flashes "Hardware out of the loop test", and RViz opens with
the arm. Some warnings are **expected** without hardware and don't mean anything is broken:

| Message | Why |
|---------|-----|
| `CAN bus 0 (can0) unavailable ... Running WITHOUT motor hardware` | No CAN HAT; the driveline keeps running and ignores motor output |
| `No SpaceMouse found under /dev/input` | No SpaceMouse plugged in; arm input is disabled |
| `No frame data (no camera connected?)` every few seconds | No camera |
| `MotorManager: LOS Safety Stop WARNING: DISABLED` every 5 seconds | No drive commands are arriving (the loss-of-signal stop is disabled in code) |
| `move_group` errors about `kinect_pointcloud` / `octomap updater` | MoveIt's default 3D-sensor config lists a Kinect that isn't installed; harmless |
| RViz `Action server: /recognize_objects not available` | An optional MoveIt RViz feature with no server; harmless |

## 6. Where to go next

- **How the pieces fit:** the [README](README.md) has the architecture diagram and a [topic map](README.md#topic-map)
  showing which node talks to which
- **Reading the code:** every source file starts with a header saying what it does, where it runs, and
  what it connects to; `// Syntax:` comments explain C++/ROS features, with more in the
  [C++ and ROS 2 primer](docs/CPP_ROS2_PRIMER.md)
- **Good first tasks:** [Known integration issues](README.md#known-integration-issues) lists real gaps,
  like making navigation able to steer the real rover, or publishing wheel speeds for the GUI
- **Before your first pull request:** read [CONTRIBUTING.md](CONTRIBUTING.md) (branching, commit messages, comment style)

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `docker: command not found` | Docker isn't installed or (on Windows) Docker Desktop isn't running; see [docs/WSL_SETUP.md](docs/WSL_SETUP.md) |
| Clean build fails in `arm_urc` with `strip: libpigpio.so ... file truncated` | Your clone has a corrupted pigpio from an older build: `git -C libs/pigpio clean -xfd`, then rebuild |
| Build fails right away with missing files in `libs/` | You skipped the submodules: `git submodule update --init --recursive` |
| GUI crashes with `Failed to open display` | Run `xhost +local:` (Linux), and make sure the `docker run` has `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix` |
| `ros2: command not found` in the dev shell | Run `source /opt/ros/humble/setup.bash` first |
| `Package 'navigation_urc' not found` | Run `source install/setup.bash` (after building) |
| Changed a `.cpp` file but nothing changed | You rebuilt in the dev shell but are running the old `urc_software` image (or vice versa); see step 4 |
| Added or removed a `.cpp` file and the build ignores it | CMake's `file(GLOB)` only re-scans on reconfigure: `rm -rf build/<package>` and rebuild |
