import paramiko
import subprocess
import argparse
import os
import threading

USERNAME = 'lusi'
PASSWORD = 'lusi'
MAIN_COMPUTER_IP = '10.0.0.10'
DRIVELINE_COMPUTER_IP = '10.0.0.20'
MAIN_COMPUTER_PATH = '/home/lusi/urc_software_deploy'
# LOCAL_PATH = '/mnt/c/Users/phamd/urc_software'
LOCAL_PATH = '/home/parallels/lusi/urc_software'
DOCKER_IMAGE_NAME = 'urc_software'

def rsync_files():
    print("Syncing files with rsync...")
    rsync_command = (
        f'rsync -avzP '
        f'--exclude=".cache/" '
        f'--exclude=".dotnet/" '
        f'--exclude=".git/" '
        f'--exclude=".gitmodules" '
        f'--exclude=".gitignore" '
        f'--exclude=".ros" '
        f'--exclude=".vscode/" '
        f'--exclude=".vscode-server/" '
        f'--exclude="build/" '
        f'--exclude="install/" '
        f'--exclude="log/" '
        f'--exclude=".bash_history" '
        f'--exclude=".gitconfig" '
        f'--exclude="imgui.ini" '
        f'--exclude="nul" '
        f'{LOCAL_PATH}/ {USERNAME}@{MAIN_COMPUTER_IP}:{MAIN_COMPUTER_PATH}/'
    )
    subprocess.run(rsync_command, shell=True)

def run_docker_build():
    print("Connecting via SSH to run Docker build...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        ssh.connect(MAIN_COMPUTER_IP, username=USERNAME, password=PASSWORD)
        print(f"Successfully connected to {MAIN_COMPUTER_IP}")
    except Exception as e:
        print(f"Failed to connect via SSH: {e}")
        return

    dos2unix_command = f'dos2unix {MAIN_COMPUTER_PATH}/run_nodes.sh'
    ssh.exec_command(dos2unix_command)

    docker_command = f'cd {MAIN_COMPUTER_PATH} && ./softwareUpdate/dockerBuild.sh'
    stdin, stdout, stderr = ssh.exec_command(docker_command)

    print("Docker build in progress...")
    for line in iter(stdout.readline, ""):
        print(line, end="")

    error_output = stderr.read().decode()
    if error_output:
        print("Docker Build Errors:\n", error_output)

    print("Pushing to Main Computer Docker Registry")

    docker_command = f'docker push localhost:65000/{DOCKER_IMAGE_NAME}'
    stdin, stdout, stderr = ssh.exec_command(docker_command)

    print("Docker Push in progress...")
    for line in iter(stdout.readline, ""):
        print(line, end="")

    error_output = stderr.read().decode()
    if error_output:
        print("Docker Push Errors:\n", error_output)

    ssh.close()


def restart_lusi_software(computer, username, password):
    print(f"Connecting via SSH to {computer} to launch software...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        ssh.connect(computer, username=username, password=password)
        print(f"Successfully connected to {computer}")
    except Exception as e:
        print(f"Failed to connect via SSH: {e}")
        return

    service_command = f'echo {password} | sudo -S systemctl restart lusi-software.service'
    stdin, stdout, stderr = ssh.exec_command(service_command)

    print("Software relaunch in progress...")
    for line in iter(stdout.readline, ""):
        print(line, end="")

    error_output = stderr.read().decode()
    if error_output:
        print("software relaunch Errors:\n", error_output)

    ssh.close()

def deploy(args):
    if args.rsync or args.docker or args.full:
        rsync_files()
    if args.docker or args.full:
        run_docker_build()
    if args.deploy or args.full:
        t1 = threading.Thread(target=restart_lusi_software,args=(MAIN_COMPUTER_IP,USERNAME,PASSWORD))
        t2 = threading.Thread(target=restart_lusi_software,args=(DRIVELINE_COMPUTER_IP,"pi",PASSWORD))
        t1.start()
        t2.start()
        t1.join()
        t2.join()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Deployment script for different deployment levels.")
    parser.add_argument('--rsync', action='store_true', help="Sync files only.")
    parser.add_argument('--docker', action='store_true', help="Sync files and build Docker image.")
    parser.add_argument('--deploy', action='store_true', help="Sync files, build Docker image and relaunch software.")
    parser.add_argument('--full', action='store_true', help="Run full deployment: sync, Docker build.")
    args = parser.parse_args()

    if not (args.rsync or args.docker or args.deploy):
        args.full = True  # Default to full deployment if no flags provided

    deploy(args)
