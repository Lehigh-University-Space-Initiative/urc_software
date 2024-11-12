#!/bin/sh

docker build -t urc_software .
# -it means the dcoekr imaage stops when the parent process stops
docker run -it --net=host -e DISPLAY=$DISPLAY -v ./ground_station_volume:/root urc_software base_station