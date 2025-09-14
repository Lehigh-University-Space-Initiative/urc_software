# URC Software

## Current Packages

### base_station_urc

Code that runs on the base station computer to control the rover.

### main_computer_urc

Code that runs on the main driveline computer.

### driveline_urc

Code running on the driveline raspberry pi which manages the drivetrain.

### arm_urc

Code running on the raspberry pi in the arm module which controls the robotic arm.

### Helper Packages

#### cross_pkg_messages

Shared ros message definitions used to communicate between packages.

#### moveit_config_urc

Some of the config file for the robotic arm's use of the ros2 moveit package.

## Running on Linux

Once per boot up of the linux machine, you need to allow remote x11 forwarding
`xhost +`

## Building

run `./src/base_station_urc/launch/launchScript.sh`

## Running

Base Station: `./src/base_station_urc/launch/launchScript.sh `

Any Package: `./src/{pkg name}/launch/launchScript.sh `


# Deploying Software

to run a full software deploy: `python3 ./softwareUpdate/urc_deploy.py `

for more options to run parts of the deploy process run: `python3 ./softwareUpdate/urc_deploy.py --help`


Alex was here, blah blah blah