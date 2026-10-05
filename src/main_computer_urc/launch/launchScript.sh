#!/bin/sh
# Starts the main computer software in the urc_software Docker image (run mode "main_computer")
#
# Flags:
#   -it                 interactive; the container stops when this terminal closes
#   --net=host          share the host's network so ROS 2 can reach the base station and the Pis
#   --ipc/--pid=host    share memory and process namespaces (lets ROS 2 use fast shared-memory transport)
#   -v /dev:/dev        expose the cameras and other devices
#   --device-cgroup-rule='c 81:* rmw'  allow access to video devices (Linux device major number 81)
#   -e DISPLAY + -v /tmp/.X11-unix     let RViz draw on your screen (the socket mount is required under WSLg)
#   --privileged        full hardware access

docker run --rm --name urc_software -it --net=host --ipc=host --pid=host -v /dev:/dev -v /dev/video0:/dev/video0 -v /dev/video1:/dev/video1 --device-cgroup-rule='c 81:* rmw' -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --privileged urc_software main_computer
