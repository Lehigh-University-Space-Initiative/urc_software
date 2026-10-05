#!/bin/sh
# Builds the urc_software Docker image and compiles every ROS 2 package in this repo
#
# Run from anywhere (it cd's to its own folder first):
#   ./softwareUpdate/dockerBuild.sh
#
# Steps:
#   1. Build the urc_software_builder image (ROS 2 Humble + every dependency; cached after the first run)
#   2. Run colcon inside that image with the repo mounted, so build/ and install/ persist on the host between builds
#   3. Copy the compiled install/ folder into the final urc_software image
#
# Exit codes: 0 success, 1 a Docker image build failed, 2 the code did not compile

# $0 is this script's path; cd'ing to its folder makes the relative paths below work from anywhere
cd "$(dirname "$0")"

docker image build -t urc_software_builder --target urc_software_builder ../
if [ $? -ne 0 ]; then
    echo "\033[31mBuilder image build failed!\033[0m"
    exit 1
fi

# Compiling the messages package first, since every other package includes its generated headers
# The repo path is quoted so a folder name containing spaces doesn't split into two arguments
docker container run \
    -v "$(pwd)/../:/ros2_ws" \
    -w /ros2_ws \
    urc_software_builder \
    bash -c 'source /opt/ros/humble/setup.sh && colcon build --merge-install --packages-select cross_pkg_messages &&
             source ./install/setup.bash &&
             colcon build --merge-install --packages-select base_station_urc main_computer_urc driveline_urc sllidar_ros2 moveit_config_urc arm_urc navigation_urc'

# Failing on any non-zero exit, not just code 2 (e.g. a missing `docker` exits 127)
if [ $? -ne 0 ]; then
    echo "\033[31mCode did not compile!\033[0m"
    exit 2
fi

# Also tagging the image for the rover's local registry (10.0.0.10:65000), which urc_deploy.py pushes to
docker image build -t urc_software -t 10.0.0.10:65000/urc_software --target urc_software ../
if [ $? -ne 0 ]; then
    echo "\033[31mFinal image build failed!\033[0m"
    exit 1
fi

echo "\033[32mCode compiled successfully!\033[0m"
