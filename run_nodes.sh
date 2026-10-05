#!/usr/bin/env bash
# Entrypoint of the urc_software Docker image: picks which ROS 2 launch file to run from the first argument
#
# Usage (the argument after the image name is the mode):
#   docker run ... urc_software <mode>
#
# Modes:
#   base_station   operator laptop: GUI, joysticks, SpaceMouse, video receiver
#   main_computer  rover main computer: MoveIt arm stack, drive relay, cameras
#   driveline      driveline Pi: wheel motors over CAN
#   arm            arm Pi: arm motors over CAN
#   rviz           RViz-only view of the arm
#   nav_sim        hardware-free navigation simulation (fake GPS + waypoint follower); needs no display or devices
#   hootl          hardware-out-of-the-loop: base station + main computer + driveline together on one machine
#   manual         an interactive bash shell inside the container (run with -it)
#
# Why "#!/usr/bin/env bash": it finds bash wherever it's installed (https://discourse.nixos.org/t/how-do-you-run-a-bash-script/10141)

# Loading the workspace built by colcon, so ros2 can find this repo's packages
source /ros2_ws/install/setup.bash

if [ -z "$DISPLAY" ]; then
    echo "Warning: DISPLAY environment variable is not set. GUI applications might not work."
fi

case "$1" in
  hootl)
    # The trailing & runs each launch in the background; wait keeps the container alive until they all exit
    ros2 launch base_station_urc base_station_launch.py hootl:=true &
    ros2 launch main_computer_urc main_computer_launch.py &
    ros2 launch driveline_urc driveline_launch.py &
    wait
    ;;
  base_station)
    ros2 launch base_station_urc base_station_launch.py
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
  nav_sim)
    ros2 launch navigation_urc navigation_sim_launch.py
    ;;
  manual)
    # exec replaces this script with bash, so the shell becomes the container's main process
    exec /bin/bash
    ;;
  *)
    echo "Unknown mode: $1. Please specify one of 'hootl', 'base_station', 'main_computer', 'rviz', 'driveline', 'arm', 'nav_sim', or 'manual'."
    exit 1
    ;;
esac
