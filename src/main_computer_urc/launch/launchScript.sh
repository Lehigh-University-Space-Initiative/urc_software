#!/bin/sh

# -it means the dcoekr imaage stops when the parent process stops
docker run -it --net=host --device /dev/video0:/dev/video0 --device /dev/video1:/dev/video1 --device-cgroup-rule='c 13:* rmw' urc_software main_computer
# docker run --rm --name urc_base_station -it --net=host -e DISPLAY=$DISPLAY -v /dev/input:/dev/input -v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 13:* rmw' -v ./ground_station_volume:/root urc_software base_station
