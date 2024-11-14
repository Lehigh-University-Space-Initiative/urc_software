# Use the official ROS 2 base image
FROM ros:humble-ros-base-jammy as build

# Install ROS 2 and other necessary packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-joy \
    libglfw3-dev \
    libglew-dev \
    libgps-dev \
    x11-apps \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /ros2_ws

# Copy the entire project
COPY ./ /ros2_ws/

# Build pigpio from the submodule
RUN cd /ros2_ws/libs/pigpio && make && make install

# Build and install the cross_pkg_messages package
RUN /bin/bash -c '. /opt/ros/humble/setup.sh && colcon build --symlink-install --packages-select cross_pkg_messages'

# Build and install the base_station_urc, main_computer_urc, and driveline_urc
RUN /bin/bash -c 'source install/setup.bash && colcon build --symlink-install --packages-select base_station_urc main_computer_urc driveline_urc ground_input_urc'


# https://medium.com/codex/a-practical-guide-to-containerize-your-c-application-with-docker-50abb197f6d4
# TODO: make multi stage docker image to dramatically reduce image size
# FROM ros:humble-ros-base-jammy

# RUN apt-get update && apt-get install -y --no-install-recommends \
#     # ros-humble-ros-base=0.10.0-1* \
#     # ros-humble-rclcpp \
#     # ros-humble-std-msgs \
#     # ros-humble-geometry-msgs \
#     # ros-humble-sensor-msgs \
#     # ros-humble-image-transport \
#     # ros-humble-cv-bridge \
#     # ros-humble-joy \
#     # libglfw3-dev \
#     # libglew-dev \
#     # libgps-dev \
#     # x11-apps \
#     # iputils-ping \
#     && rm -rf /var/lib/apt/lists/*

# COPY --from=build ./ros2_ws/ ./ros2_ws/
# COPY --from=build ./ros2_ws/build ./ros2_ws/build
# COPY --from=build ./ros2_ws/install ./ros2_ws/install
# COPY --from=build ./ros2_ws/run_nodes.sh ./ros2_ws/run_nodes.sh

# TODO: @TINA why are these after the build????
# Set environment variables for build
ENV CPLUS_INCLUDE_PATH=/ros2_ws/src/base_station_urc/include/cs_libguarded:$CPLUS_INCLUDE_PATH
ENV AMENT_PREFIX_PATH=/ros2_ws/install/ground_input_urc:/ros2_ws/install/driveline_urc:/ros2_ws/install/main_computer_urc:/ros2_ws/install/base_station_urc:/ros2_ws/install/cross_pkg_messages:/opt/ros/humble:$AMENT_PREFIX_PATH
ENV CMAKE_PREFIX_PATH=/ros2_ws/install/cross_pkg_messages:$CMAKE_PREFIX_PATH

# Source the workspace in bashrc
RUN echo "export AMENT_PREFIX_PATH=/ros2_ws/install/ground_input_urc:/ros2_ws/install/driveline_urc:/ros2_ws/install/main_computer_urc:/ros2_ws/install/base_station_urc:/ros2_ws/install/cross_pkg_messages:/opt/ros/humble:\$AMENT_PREFIX_PATH" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Copy the urcAssets directory to the home directory in the container
RUN mkdir -p /home/urcAssets
COPY urcAssets /home/urcAssets

# Default command
ENTRYPOINT ["/ros2_ws/run_nodes.sh"]
# ENTRYPOINT ["bash"]
