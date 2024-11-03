import paramiko
import subprocess
import os
import time

USERNAME = 'lusi'
PASSWORD = 'lusi'
MAIN_COMPUTER_IP = '10.0.0.10'
MAIN_COMPUTER_PATH = '/home/lusi/urc_software_deploy'
LOCAL_PATH = '/mnt/c/Users/phamd/urc_software'
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
        f'--exclude="softwareUpdate/" '
        f'--exclude=".bash_history" '
        f'--exclude=".gitconfig" '
        f'--exclude="imgui.ini" '
        f'--exclude="nul" '
        f'{LOCAL_PATH}/ {USERNAME}@{MAIN_COMPUTER_IP}:{MAIN_COMPUTER_PATH}/'
    )
    subprocess.run(rsync_command, shell=True)

# SSH connection and run the Docker build command
def run_docker_build():
    print("Connecting via SSH to run Docker build...")

    # Establish an SSH connection
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        ssh.connect(MAIN_COMPUTER_IP, username=USERNAME, password=PASSWORD)
        print(f"Successfully connected to {MAIN_COMPUTER_IP}")
    except Exception as e:
        print(f"Failed to connect via SSH: {e}")
        return

    # Run dos2unix on run_nodes.sh
    dos2unix_command = f'dos2unix {MAIN_COMPUTER_PATH}/run_nodes.sh'
    ssh.exec_command(dos2unix_command)

    # Run Docker build
    docker_command = f'cd {MAIN_COMPUTER_PATH} && docker build -t {DOCKER_IMAGE_NAME} .'
    
    stdin, stdout, stderr = ssh.exec_command(docker_command)

    print("Docker build in progress...")

    # Iterate through each line of the output
    for line in iter(stdout.readline, ""):
        print(line, end="")

    error_output = stderr.read().decode()
    if error_output:
        print("Docker Build Errors:\n", error_output)

    ssh.close()

# Main deployment function
def deploy():
    rsync_files()

    # SSH into and run Docker build
    run_docker_build()

if __name__ == "__main__":
    deploy()
