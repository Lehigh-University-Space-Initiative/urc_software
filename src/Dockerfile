# Use the official ROS 2 base image
FROM ros:humble-ros-base-jammy

# install specific ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-joy \
    && rm -rf /var/lib/apt/lists/*

# Copy ROS 2 workspace
WORKDIR /ros2_ws
COPY ./base_station_urc /ros2_ws/src/base_station_urc

# Build the ROS 2 workspace
RUN /bin/bash -c '. /opt/ros/humble/setup.sh && colcon build --symlink-install'

# Source the workspace
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command to run when starting the container
CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 launch base_station_urc base_station_launch.py"]
