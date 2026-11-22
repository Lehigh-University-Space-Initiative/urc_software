#!/bin/sh
# Starts the browser interface on the base station (run mode "web_ui")
#
# Flags:
#   -it          interactive; the container stops when this terminal closes
#   --net=host   share the base station's network, so the Steam Deck can reach ports 8000, 8080, and 9090
#   -v ./ground_station_volume:/root   keep saved Science snapshots in ./ground_station_volume/science_data
#
# Then open http://<base-station-ip>:8000 in a browser (Steam Deck or laptop on the same network)

docker run --rm --name urc_web_ui -it --net=host -v ./ground_station_volume:/root urc_software web_ui
