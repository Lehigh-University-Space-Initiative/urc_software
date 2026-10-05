#!/bin/sh
# Starts the base station software in the urc_software Docker image (run mode "base_station")
#
# Flags:
#   -it                 interactive; the container stops when this terminal closes
#   --net=host          share the host's network so ROS 2 can reach the rover
#   --ipc/--pid=host    share memory and process namespaces with the host (used by ROS 2's shared-memory transport)
#   -e DISPLAY + -v /tmp/.X11-unix  let the GUI draw on your screen (the socket mount is required under WSLg)
#   -v /dev/input, -v /dev/bus/usb  expose the joysticks and SpaceMouse
#   --device-cgroup-rule='c 13:* rmw'  allow access to input devices (Linux device major number 13)
#   -v ./ground_station_volume:/root  keep the GUI layout (ui.ini) between runs, in ./ground_station_volume
#
# USB passthrough background: https://stackoverflow.com/questions/73485023/pyjoystick-inside-docker-container

docker run --rm --name urc_base_station -it --net=host --ipc=host --pid=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev/input:/dev/input -v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 13:* rmw' -v ./ground_station_volume:/root urc_software base_station
