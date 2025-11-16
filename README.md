# URC Software

## Software Setup
### Linux
*Note that this has only been tested on Ubuntu 22.04, but should work on any distro*
1. Clone the repo onto your computer
2. In the top level of the repository, run:
```bash
git submodule init
git submodule update
```

This will install the repositories for external libraries including the GUI (ImGUI), Pi pin access (PiGPIO), sockets (sockpp), LiDAR (slider_ros2), and code documentation generation (Doxygen)

3. Run 
```bash
./softwareUpdate/dockerBuild.sh
```

This will build the docker image. Wait for this to complete.

4. To run the base station software, run
```bash
xhost +
```
To let the GUI show up on your display, then:
```bash
./src/base_station_urc/launch/launchScript.sh
```

### Windows
(WIP)

### MacOS
(WIP)

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

#Starting with Docker in case ros2 command doesnt work:
```bash
docker exec -it urc_base_station bash
```