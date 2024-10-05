import paramiko
import subprocess
import os
import time
import socket

USERNAME = 'lusi'
MAIN_COMPUTER_IP = '10.1.0.4'
MAIN_COMPUTER_PATH = '/home/lusi'
LOCAL_PATH = '/mnt/c/Users/phamd/urc_software'

#instead of scp use rsync with the -a: archive mode, -v: verbose, -z: compress file data during the transfer, -e: specify the remote shell to use (ssh), -P: to show progress
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

# Main deployment function for testing
def deploy():
    # Rsync files to the "main computer" directory
    rsync_files()

if __name__ == "__main__":
    deploy()