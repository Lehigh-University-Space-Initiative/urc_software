"""
Deploys this repo to the rover: copies the code, builds the Docker image on the main computer, and restarts the software

Stages (each flag runs exactly what's listed):
    --rsync   copy the repo to the main computer (rsync over SSH)
    --docker  copy, then build the image on the main computer and push it to the rover's registry (10.0.0.10:65000)
    --deploy  only restart lusi-software.service on the main computer and the driveline Pi (no copy, no build)
    --full    copy, build/push, then restart (the default when no flag is given)

Run it with:
    python3 ./softwareUpdate/urc_deploy.py
    python3 ./softwareUpdate/urc_deploy.py --docker

Requires:
    - The deploying laptop on the rover network (10.0.0.x), with rsync installed
    - The paramiko Python package (pip install paramiko)

Note: the SSH username/password are hardcoded below; anyone with repo access can read them
"""

# Imports

import paramiko
import subprocess
import argparse
import os
import threading
import pathlib

USERNAME = "lusi"
PASSWORD = "lusi"
MAIN_COMPUTER_IP = "10.0.0.10"
DRIVELINE_COMPUTER_IP = "10.0.0.20"
MAIN_COMPUTER_PATH = "/home/lusi/urc_software_deploy"  # Where the repo is copied to on the main computer
LOCAL_PATH = pathlib.Path(__file__).parent.parent.resolve()  # This repo's root (the folder above softwareUpdate/)
DOCKER_IMAGE_NAME = "urc_software"




# ---------------------------------
# Deploy stages
# ---------------------------------

def rsync_files():
    """
    Copy the repo to the main computer, skipping build output, git data, and editor files

    rsync only sends files that changed, so repeat deploys are fast
    """
    print("Syncing files with rsync...")
    # -a keeps permissions and timestamps, -v lists files, -z compresses in transit, -P shows progress
    rsync_command = (
        f"rsync -avzP "
        f"--exclude=\".cache/\" "
        f"--exclude=\".dotnet/\" "
        f"--exclude=\".git/\" "
        f"--exclude=\".gitmodules\" "
        f"--exclude=\".gitignore\" "
        f"--exclude=\".ros\" "
        f"--exclude=\".vscode/\" "
        f"--exclude=\".vscode-server/\" "
        f"--exclude=\"build/\" "
        f"--exclude=\"install/\" "
        f"--exclude=\"log/\" "
        f"--exclude=\".bash_history\" "
        f"--exclude=\".gitconfig\" "
        f"--exclude=\"imgui.ini\" "
        f"--exclude=\"nul\" "
        f"--exclude=\"libs/pigpio/*.so*\" "
        f"--exclude=\"libs/pigpio/*.o\" "
        f"{LOCAL_PATH}/ {USERNAME}@{MAIN_COMPUTER_IP}:{MAIN_COMPUTER_PATH}/"
    )
    subprocess.run(rsync_command, shell=True)


def run_docker_build():
    """
    SSH into the main computer, run dockerBuild.sh there, then push the image to the rover's registry

    Returns:
        The remote dockerBuild.sh exit code (0 on success), or None if the SSH connection failed

    Steps:
        1. Connect over SSH and convert run_nodes.sh to Linux line endings (in case it was saved with Windows CRLF)
        2. Run dockerBuild.sh, streaming its output here, and read its exit code
        3. If it succeeded, push the image to the rover's registry so other rover computers can pull it
    """
    print("Connecting via SSH to run Docker build...")
    ssh = paramiko.SSHClient()
    # Accepting unknown host keys automatically (convenient on the rover network, but it skips host verification)
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        ssh.connect(MAIN_COMPUTER_IP, username=USERNAME, password=PASSWORD)
        print(f"Successfully connected to {MAIN_COMPUTER_IP}")
    except Exception as e:
        print(f"Failed to connect via SSH: {e}")
        return None

    dos2unixCommand = f"dos2unix {MAIN_COMPUTER_PATH}/run_nodes.sh"
    ssh.exec_command(dos2unixCommand)

    dockerCommand = f"cd {MAIN_COMPUTER_PATH} && ./softwareUpdate/dockerBuild.sh"
    stdin, stdout, stderr = ssh.exec_command(dockerCommand)

    print("Docker build in progress...")
    # Syntax: iter(stdout.readline, "") keeps calling readline until it returns "" (end of output)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    buildExitCode = stdout.channel.recv_exit_status()  # Waits for the remote command to finish
    if buildExitCode != 0:
        ssh.close()
        return buildExitCode

    print("Pushing to Main Computer Docker Registry")

    dockerCommand = f"docker push 10.0.0.10:65000/{DOCKER_IMAGE_NAME}"
    stdin, stdout, stderr = ssh.exec_command(dockerCommand)

    print("Docker Push in progress...")
    for line in iter(stdout.readline, ""):
        print(line, end="")

    errorOutput = stderr.read().decode()
    if errorOutput:
        print("Docker Push Errors:\n", errorOutput)

    ssh.close()

    return 0


def restart_lusi_software(computer, username, password):
    """
    Restart the lusi-software systemd service on one rover computer

    Args:
        computer: IP address of the computer
        username: SSH username
        password: SSH password (also used for sudo)
    """
    print(f"Connecting via SSH to {computer} to launch software...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        ssh.connect(computer, username=username, password=password)
        print(f"Successfully connected to {computer}")
    except Exception as e:
        print(f"Failed to connect via SSH: {e}")
        return

    # sudo -S reads the password from stdin, which echo provides
    serviceCommand = f"echo {password} | sudo -S systemctl restart lusi-software.service"
    stdin, stdout, stderr = ssh.exec_command(serviceCommand)

    print("Software relaunch in progress...")
    for line in iter(stdout.readline, ""):
        print(line, end="")

    errorOutput = stderr.read().decode()
    if errorOutput:
        print("software relaunch Errors:\n", errorOutput)

    ssh.close()




# ---------------------------------
# Entry point
# ---------------------------------

def deploy(args):
    """
    Run the deploy stages selected by the command-line flags

    Args:
        args: parsed flags (rsync, docker, deploy, full)
    """
    if args.rsync or args.docker or args.full:
        rsync_files()

    if args.docker or args.full:
        buildExitCode = run_docker_build()
        if buildExitCode != 0:
            print(f"Build failed (exit code {buildExitCode}); not restarting the rover software")
            return

    if args.deploy or args.full:
        # Restarting both computers at the same time on separate threads
        t1 = threading.Thread(target=restart_lusi_software, args=(MAIN_COMPUTER_IP, USERNAME, PASSWORD))
        t2 = threading.Thread(target=restart_lusi_software, args=(DRIVELINE_COMPUTER_IP, "pi", PASSWORD))
        t1.start()
        t2.start()
        t1.join()
        t2.join()


# Syntax: this block only runs when the file is executed directly (python3 urc_deploy.py), not when imported
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Deployment script for different deployment levels.")
    parser.add_argument("--rsync", action="store_true", help="Sync files only.")
    parser.add_argument("--docker", action="store_true", help="Sync files, build the Docker image, and push it.")
    parser.add_argument("--deploy", action="store_true", help="Only restart the rover software (no sync, no build).")
    parser.add_argument("--full", action="store_true", help="Sync, build and push, then restart (default).")
    args = parser.parse_args()

    if not (args.rsync or args.docker or args.deploy):
        args.full = True  # Defaulting to a full deployment when no flags are given

    deploy(args)
