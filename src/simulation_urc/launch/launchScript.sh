#!/bin/sh

docker run --rm --name urc_simulation -it --network=host --ipc=host --pid=host -e DISPLAY=$DISPLAY urc_software simulation
# -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR -e QT_X11_NO_MITSHM=1   -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp   -e ROS_DOMAIN_ID=0   -v /tmp/.X11-unix:/tmp/.X11-unix   -v /dev/input:/dev/input   -v /dev/bus/usb:/dev/bus/usb   --device-cgroup-rule='c 13:* rmw'   -v "$PWD/ground_station_volume":/root   urc_software base_station
