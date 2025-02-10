# https://github.com/pcewing/docker-incremental-compile-demo
# Use the official ROS 2 base image
FROM ros:humble-ros-base-jammy AS urc_software_base

# Install ROS 2 and other necessary packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    ros-humble-joy \
    ros-humble-xacro \
    ros-humble-moveit \
    ros-humble-joint-state-publisher-gui \
    libqt5widgets5 \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-servo \
    tmux \
    ruby \
    vim nano gdb \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    libglfw3-dev \
    libglew-dev \
    libgps-dev \
    iproute2 net-tools \
    x11-apps \
    iputils-ping \
    # for opencv
     wget g++ unzip \ 
    && rm -rf /var/lib/apt/lists/*

# Copy the urcAssets directory to the home directory in the container
RUN mkdir -p /home/urcAssets
COPY urcAssets /home/urcAssets

FROM urc_software_base AS urc_software_builder

# Set the working directory
WORKDIR /ros2_ws

# Copy the libs
COPY ./libs /ros2_ws/libs

# Build pigpio from the submodule
RUN cd /ros2_ws/libs/pigpio && make && make install

# https://medium.com/codex/a-practical-guide-to-containerize-your-c-application-with-docker-50abb197f6d4
FROM urc_software_base AS urc_software 

# copy built binaries
WORKDIR /ros2_ws
COPY --from=urc_software_builder /opt/ros/humble /opt/ros/humble
# for installing Qt for rviz2
COPY ./install /ros2_ws/install
COPY ./libs /ros2_ws/libs
COPY ./run_nodes.sh /ros2_ws/run_nodes.sh

RUN cd /ros2_ws/libs/pigpio && make && make install

# Default command
ENTRYPOINT ["/ros2_ws/run_nodes.sh"] 
#  ENTRYPOINT ["bash"]

# added
