#!/bin/sh

docker run --rm --name urc_rover_description \
    -it --net=host --ipc=host --pid=host \
    urc_software rover_description