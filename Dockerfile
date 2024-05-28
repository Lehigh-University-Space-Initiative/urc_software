# Use the official ROS 2 base image
FROM ros:humble-ros-base-jammy

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
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /ros2_ws

# Copy the entire project
COPY ./ /ros2_ws/

# Build the ROS 2 workspace
RUN /bin/bash -c '. /opt/ros/humble/setup.sh && colcon build --packages-select base_station_urc --symlink-install'

# Source the workspace in bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command to run when starting the container
CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 launch base_station_urc base_station_launch.py"]
