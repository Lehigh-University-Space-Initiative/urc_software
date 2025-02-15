#!/bin/sh

docker run --rm --name urc_arm -it --net=host urc_software arm
# docker run --rm --name urc_base_station -it --net=host -e DISPLAY=$DISPLAY --device=/dev/input/js1 -v ./ground_station_volume:/root urc_software base_station