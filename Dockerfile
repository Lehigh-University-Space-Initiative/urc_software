# Use ROS 2 Humble base image
FROM ros:humble-ros-base-jammy

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /ros2_ws

# Copy the libs
COPY ./libs /ros2_ws/libs

# # Build pigpio from the submodule
RUN cd /ros2_ws/libs/pigpio && make && make install

# Copy the package into the container
COPY src/driveline /ros2_ws/src/driveline_urc
COPY src/cross_pkg_messages_urc /ros2_ws/src/cross_pkg_messages_urc

# # Set the library path in the CMake configuration
# ENV CMAKE_PREFIX_PATH=/workspace/libs/pigpio:$CMAKE_PREFIX_PATH

# # Build the custom dependency package
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select cross_pkg_messages driveline_urc

# # Source the workspace by default
# RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Set the default command
CMD ["bash"]
