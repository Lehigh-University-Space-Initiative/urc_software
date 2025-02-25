#!/bin/sh

cd "$(dirname "$0")"

# build the build environment docker image
docker image build -t urc_software_builder --target urc_software_builder ../

# do the real build pulling in build/ from the host to save build progress
docker container run \
    -v $(pwd)/../:/ros2_ws \
    -w /ros2_ws \
    urc_software_builder \
    bash -c 'source /opt/ros/humble/setup.sh && colcon build --merge-install --packages-select cross_pkg_messages &&
             source ./install/setup.bash &&
             colcon build --merge-install --packages-select base_station_urc main_computer_urc driveline_urc moveit_config_urc arm_urc'

# Check if the build step failed (return code 2)
if [ $? -eq 2 ]; then
    echo "\033[31mCode did not compile!\033[0m"
    exit 2
fi

# copy the built binaries into the final
docker image build -t urc_software -t 10.0.0.10:65000/urc_software --target urc_software ../

echo "\033[32mCode compiled successfully!\033[0m"
