#!/bin/sh
# Runs the hardware-free navigation simulation in the urc_software Docker image (run mode "nav_sim")
#
# This needs no rover, no display, and no devices, so it is the quickest way to see the software working
# Watch WaypointFollower's "distance=" log shrink until it prints "Arrived at target"; press Ctrl+C to stop
#
# Flags:
#   -it          interactive; Ctrl+C stops the simulation and the container
#   --net=host   share the host's network (lets you inspect the topics with ros2 CLI tools from another container)

docker run --rm --name urc_nav_sim -it --net=host urc_software nav_sim
