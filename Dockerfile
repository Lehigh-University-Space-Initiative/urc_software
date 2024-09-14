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
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /ros2_ws

# Copy the entire project
COPY ./ /ros2_ws/

# Build and install the cross_pkg_messages package
RUN /bin/bash -c '. /opt/ros/humble/setup.sh && colcon build --symlink-install --packages-select cross_pkg_messages'

# Build and install the base_station_urc package
RUN /bin/bash -c 'source install/setup.bash && colcon build --symlink-install --packages-select base_station_urc'

# Build and install the main_computer_urc package
RUN /bin/bash -c 'source install/setup.bash && colcon build --symlink-install --packages-select main_computer_urc'

# Set environment variables for build
ENV CPLUS_INCLUDE_PATH=/ros2_ws/src/base_station_urc/include/cs_libguarded:$CPLUS_INCLUDE_PATH
ENV AMENT_PREFIX_PATH=/ros2_ws/install/main_computer_urc:/ros2_ws/install/cross_pkg_messages:/opt/ros/humble:$AMENT_PREFIX_PATH
ENV CMAKE_PREFIX_PATH=/ros2_ws/install/cross_pkg_messages:$CMAKE_PREFIX_PATH

# Source the workspace in bashrc
RUN echo "export AMENT_PREFIX_PATH=/ros2_ws/install/main_computer_urc:/ros2_ws/install/cross_pkg_messages:/opt/ros/humble:\$AMENT_PREFIX_PATH" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Source the workspace in bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Copy the urcAssets directory to the home directory in the container
RUN mkdir -p /home/urcAssets
COPY urcAssets /home/urcAssets

# Default command to run when starting the container
CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 launch main_computer_urc main_computer_launch.py & ros2 run base_station_urc base_station_urc.py && wait"]
