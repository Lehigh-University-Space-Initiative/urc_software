#!/bin/sh
# Starts the arm software in the urc_software Docker image (run mode "arm")
#
# Flags:
#   -it                 interactive; the container stops when this terminal closes
#   --net=host          share the host's network, which also exposes the Pi's can1 interface to the container
#   --ipc/--pid=host    share memory and process namespaces with the host (used by ROS 2's shared-memory transport)
#
# Without the CAN HAT (e.g. on a laptop) the node logs "CAN bus 1 (can1) unavailable" once and keeps running

docker run --rm --name urc_arm -it --net=host --ipc=host --pid=host urc_software arm
