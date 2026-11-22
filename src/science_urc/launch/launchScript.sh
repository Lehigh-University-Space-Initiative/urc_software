#!/bin/sh
# Starts the Science Module software on the Science Raspberry Pi (run mode "science")
#
# Flags:
#   -it          interactive; the container stops when this terminal closes
#   --net=host   share the Pi's network so ROS can reach the main computer, and so the
#                motor node can reach the pigpio daemon on localhost
#   --privileged gives the container the serial port, GPIO, and USB cameras
#                (VERIFY on the Pi: replace with specific --device flags once the device list is known)
#
# Before running: start the pigpio daemon on the Pi (sudo pigpiod), which science_node needs

docker run --rm --name urc_science -it --net=host --privileged urc_software science
