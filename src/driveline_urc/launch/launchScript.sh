#!/bin/sh
# Starts the driveline software in the urc_software Docker image (run mode "driveline")
#
# Flags:
#   -it          interactive; the container stops when this terminal closes
#   --net=host   share the host's network, which also exposes the Pi's can0 interface to the container
#
# Without the CAN HAT (e.g. on a laptop) the node logs "CAN bus 0 (can0) unavailable" once and keeps running

docker run --rm --name urc_driveline -it --net=host urc_software driveline
