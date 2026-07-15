# Building & testing on Windows with WSL 2

The rover software is Linux/Docker only. On a Windows machine the supported way to
**compile the workspace** and **run the GUI / simulation** is
[WSL 2](https://learn.microsoft.com/windows/wsl/) (Windows Subsystem for Linux)
with Docker.

## What WSL can and cannot do

| Works in WSL 2 | Does **not** work in WSL |
| --------------- | -------------------------- |
| `./softwareUpdate/dockerBuild.sh` — full compile of every package | Real CAN bus / SparkMax motor control |
| Base-station GUI (ImGui) via WSLg X11 | Raspberry Pi GPIO (`pigpio` fails at runtime off-Pi) |
| `hootl` simulation mode (base station + main computer + driveline together) | Physical joysticks / USB cameras (unless USB is forwarded) |
| RViz / MoveIt visualization | The `10.0.0.x` rover network and `urc_deploy.py` targets |

Use WSL for **compile verification and simulation**. Hardware-in-the-loop testing
still has to happen on the actual rover computers (or the team's Ubuntu VM).

## Prerequisites

- Windows 11 (or Windows 10 21H2+). WSLg (GUI support) is built in on Windows 11.
- Administrator rights and one reboot for the initial install.
- **At least 20–30 GB free disk space, kept free throughout** — the Docker image,
  build cache, and WSL's virtual disk all grow as you build and rebuild. If your
  Windows drive fills up completely, the WSL filesystem itself can corrupt (see
  [Recovering from a full disk](#recovering-from-a-full-disk)). Check available
  space again after your first build, not just before you start.

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
wsl --list --verbose      # VERSION should be 2
```

`STATE` shows `Stopped` whenever nothing is actively using the distro — that is
normal, not a failure. It flips to `Running` as soon as you open a shell in it
(or as soon as any command targets it, e.g. `wsl -d Ubuntu-22.04 -- <command>`).

## 2. Install Docker

Two options — pick one.

**Option A — Docker Desktop (simplest).**
Install [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/).
In **Settings → Resources → WSL integration**, enable integration with your
`Ubuntu-22.04` distro, then **Apply & Restart**.

> **Docker Desktop has to actually be running, not just installed.** Turning on
> WSL integration is not enough by itself — the Docker Desktop app needs to be
> open. If `docker` works one moment and then starts failing with
> `docker: command not found` later in the same session with no changes on your
> end, Docker Desktop's **Resource Saver** mode has almost certainly suspended
> its backend after a period of inactivity. Open the Docker Desktop window from
> the system tray to wake it back up — commands run from a WSL terminal do not
> reliably trigger that wake-up on their own. If this interrupts your workflow
> often, go to **Settings → Resources → Resource Saver** and raise the idle
> timer, or disable it.

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

> **If this fails with `exec format error` mentioning
> `docker-credential-desktop.exe`:** your `~/.docker/config.json` points at a
> Windows credential helper that Linux can't execute. Fix it with:
>
> ```bash
> echo '{}' > ~/.docker/config.json
> ```
>
> This only disables registry *login* caching — pulling public images, which is
> all this project needs, still works fine afterward.

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

A successful run prints **"Code compiled successfully!"** — but that message is
currently printed unconditionally, even if the final Docker image build step
failed. **Don't trust the message alone.** Confirm the image actually updated:

```bash
docker images    # check urc_software:latest exists and is recent
```

If it's missing, or looks stale after a build that should have changed it,
re-run just the final packaging step to see the real error:

```bash
docker image build -t urc_software --target urc_software .
```

A common cause: this step ran before `colcon build`'s `install/` output had
finished writing to disk. Re-running it once the build has fully settled
resolves it.

## 4. Run the GUI / simulation

WSLg provides an X server automatically, so `$DISPLAY` is already set — you do
**not** need a third-party X server. From the repo root:

```bash
# Allow the container to use the display (once per session)
xhost +local:            # if 'xhost' is missing: sudo apt-get install -y x11-xserver-utils

# Base station GUI
./src/base_station_urc/launch/launchScript.sh

# Or the full hardware-out-of-the-loop simulation:
docker run --rm -it --net=host --ipc=host --pid=host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  urc_software hootl
```

The ImGui window and RViz should appear as normal Windows windows.

> `--net=host` shares the container's network with WSL but **not** its
> filesystem, so the X11 socket WSLg exposes at `/tmp/.X11-unix` must be
> mounted explicitly with `-v`. Without it, the GUI process crashes on launch
> with `GLFW Error 65544: X11: Failed to open display :0`. `launchScript.sh`
> already includes this mount — only add it yourself if you're invoking
> `docker run` directly, as in the `hootl` command above.

## Tips & troubleshooting

- **GUI window never appears / `GLFW Error ...: X11: Failed to open display`:**
  confirm WSLg is active with `echo $DISPLAY` (should print something like
  `:0`). If it's set but the error persists, you're likely missing the
  `-v /tmp/.X11-unix:/tmp/.X11-unix` mount — see step 4. If `$DISPLAY` is
  empty, run `wsl --update` from PowerShell.
- **`docker: command not found` in WSL:** either Docker Desktop's WSL
  integration is off (Option A), the Docker service isn't started (Option B:
  `sudo service docker start`), or — if it was working earlier in the same
  session — Docker Desktop's backend has gone to sleep (see step 2).
- **`docker run` fails with `exec format error` on `docker-credential-desktop.exe`:**
  see the credential helper fix in step 2.
- **`dockerBuild.sh` prints "Code compiled successfully!" but nothing changed:**
  the script doesn't check the exit code of its final image-build step. Verify
  manually with `docker images` — see step 3.
- **Slow builds / file-watching issues:** you are probably building under
  `/mnt/c/...`. Move the clone into the Linux home directory.
- **USB devices (joystick/camera):** WSL does not pass USB through by default. If
  you need it, see [usbipd-win](https://learn.microsoft.com/windows/wsl/connect-usb);
  otherwise stick to simulation.
- **clang-format:** install with `sudo apt-get install -y clang-format` inside the
  distro to run the formatter described in [CONTRIBUTING.md](CONTRIBUTING.md).

### Recovering from a full disk

If your Windows drive fills up completely, WSL's virtual disk can't grow to
accept new writes and the guest Linux filesystem can wedge mid-operation.
Symptoms include VS Code popping up **"VS Code Server for WSL closed
unexpectedly"**, and basic commands failing with `Input/output error` instead
of running normally.

1. Free up space on the Windows drive. Docker's image layers and build cache
   are common culprits — once `docker` is responsive again,
   `docker system prune` reclaims space from old layers.
2. From PowerShell, fully shut down WSL — closing the terminal window is not
   enough:

   ```powershell
   wsl --shutdown
   ```

3. Reopen your WSL terminal or VS Code window. The distro restarts cleanly.
4. Confirm your Docker images survived with `docker images` — they usually do.
