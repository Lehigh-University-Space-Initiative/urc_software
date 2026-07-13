# Building & testing on Windows with WSL 2

The rover software is Linux/Docker only. On a Windows machine the supported way to
**compile the workspace** and **run the GUI / simulation** is
[WSL 2](https://learn.microsoft.com/windows/wsl/) (Windows Subsystem for Linux)
with Docker.

## What WSL can and cannot do

| Works in WSL 2 | Does **not** work in WSL |
|----------------|--------------------------|
| `./softwareUpdate/dockerBuild.sh` — full compile of every package | Real CAN bus / SparkMax motor control |
| Base-station GUI (ImGui) via WSLg X11 | Raspberry Pi GPIO (`pigpio` fails at runtime off-Pi) |
| `hootl` simulation mode (base station + main computer + driveline together) | Physical joysticks / USB cameras (unless USB is forwarded) |
| RViz / MoveIt visualization | The `10.0.0.x` rover network and `urc_deploy.py` targets |

Use WSL for **compile verification and simulation**. Hardware-in-the-loop testing
still has to happen on the actual rover computers (or the team's Ubuntu VM).

## Prerequisites

- Windows 11 (or Windows 10 21H2+). WSLg (GUI support) is built in on Windows 11.
- Administrator rights and one reboot for the initial install.
- ~15 GB free disk for the Docker image and build cache.

## 1. Install WSL 2 + Ubuntu

Open **PowerShell as Administrator** and run:

```powershell
wsl --install -d Ubuntu-22.04
```

This enables the required Windows features, installs the WSL 2 kernel, and
installs Ubuntu 22.04 (the distro the software is tested on). **Reboot** when
prompted, then launch **Ubuntu** from the Start menu and create your Linux
username/password.

Verify you are on version 2:

```powershell
wsl --list --verbose      # STATE should be Running, VERSION should be 2
```

## 2. Install Docker

Two options — pick one.

**Option A — Docker Desktop (simplest).**
Install [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/),
then in **Settings → Resources → WSL integration** enable integration with your
`Ubuntu-22.04` distro. `docker` is then available from inside WSL.

**Option B — Docker Engine inside the distro (no Docker Desktop).**
Inside the Ubuntu shell:

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER      # run docker without sudo
```

Then start the daemon (WSL has no systemd by default unless enabled):

```bash
sudo service docker start
```

> To get `sudo service docker start` to persist / use systemd, add
> `[boot]\nsystemd=true` to `/etc/wsl.conf` and run `wsl --shutdown` from
> PowerShell to restart the distro.

Confirm Docker works:

```bash
docker run --rm hello-world
```

## 3. Clone and build

Work inside the **Linux filesystem** (e.g. `~/`), **not** `/mnt/c/...` — building
on the mounted Windows drive is dramatically slower and has permission quirks.

```bash
cd ~
git clone <your-repo-url> urc_software
cd urc_software
git submodule update --init --recursive
./softwareUpdate/dockerBuild.sh
```

A successful run prints **"Code compiled successfully!"**. This is the fastest way
to verify a change compiles.

## 4. Run the GUI / simulation

WSLg provides an X server automatically, so `$DISPLAY` is already set — you do
**not** need a third-party X server. From the repo root:

```bash
# Allow the container to use the display (once per session)
xhost +local:            # if 'xhost' is missing: sudo apt-get install -y x11-xserver-utils

# Base station GUI
./src/base_station_urc/launch/launchScript.sh

# Or the full hardware-out-of-the-loop simulation:
docker run --rm -it --net=host --ipc=host --pid=host -e DISPLAY=$DISPLAY urc_software hootl
```

The ImGui window and RViz should appear as normal Windows windows.

## Tips & troubleshooting

- **GUI window never appears:** confirm WSLg is active with `echo $DISPLAY`
  (should print something like `:0`). Update WSL with `wsl --update` from
  PowerShell if it is empty.
- **`docker: command not found` in WSL:** Docker Desktop WSL integration is off
  (Option A), or the Docker service isn't started (Option B:
  `sudo service docker start`).
- **Slow builds / file-watching issues:** you are probably building under
  `/mnt/c/...`. Move the clone into the Linux home directory.
- **USB devices (joystick/camera):** WSL does not pass USB through by default. If
  you need it, see [usbipd-win](https://learn.microsoft.com/windows/wsl/connect-usb);
  otherwise stick to simulation.
- **clang-format:** install with `sudo apt-get install -y clang-format` inside the
  distro to run the formatter described in [CONTRIBUTING.md](CONTRIBUTING.md).
