#!/bin/sh

# docker build -t urc_software .

# for passing usb devices: https://stackoverflow.com/questions/73485023/pyjoystick-inside-docker-container

# -it means the dcoekr imaage stops when the parent process stops
docker run --rm --name urc_base_station -it --net=host --ipc=host --pid=host -e DISPLAY=$DISPLAY -v /dev/input:/dev/input -v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 13:* rmw' -v ./ground_station_volume:/root urc_software base_station
# docker run --rm --name urc_base_station -it --net=host -e DISPLAY=$DISPLAY --device=/dev/input/js1 -v ./ground_station_volume:/root urc_software base_station