#!/usr/bin/env bash 
# ^^ https://discourse.nixos.org/t/how-do-you-run-a-bash-script/10141

# Source the ROS 2 setup
source /ros2_ws/install/setup.bash

# Check if the DISPLAY environment variable is set
if [ -z "$DISPLAY" ]; then
    echo "Warning: DISPLAY environment variable is not set. GUI applications might not work."
fi

# Check which mode to run based on the argument passed to docker run
case "$1" in
  hootl)
    ros2 launch base_station_urc base_station_launch.py &
    ros2 launch main_computer_urc main_computer_launch.py &
    ros2 launch driveline_urc driveline_launch.py &
    wait
    ;;
  base_station)
    ros2 launch base_station_urc base_station_launch.py
    # ros2 run base_station_urc GroundStationGUI
    ;;
  main_computer)
    ros2 launch main_computer_urc main_computer_launch.py gui_only:=false
    ;;
  rviz)
    ros2 launch main_computer_urc rviz_gui_launch.py
    ;;
  driveline)
    ros2 launch driveline_urc driveline_launch.py
    ;;
  arm)
    ros2 launch arm_urc arm_launch.py
    ;;
  ground_input)
    ros2 launch ground_input_urc ground_input_launch.py
    ;;
  manual)
    # run the docker container with -it flags for this to work
    exec /bin/bash
    ;;
  lusi_training)
    ros2 launch lusi_training training_launch.py
    ;;
  *)
    echo "Unknown mode: $1. Please specify one of 'hootl', 'base_station', 'main_computer', or 'driveline'."
    exit 1
    ;;
esac
