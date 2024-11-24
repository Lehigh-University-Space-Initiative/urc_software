#!/bin/sh

# -it means the dcoekr imaage stops when the parent process stops
docker run -it --net=host -v /dev/input:/dev/input -v /dev/bus/usb:/dev/bus/usb urc_software main_computer
# docker run --rm --name urc_base_station -it --net=host -e DISPLAY=$DISPLAY -v /dev/input:/dev/input -v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 13:* rmw' -v ./ground_station_volume:/root urc_software base_station
