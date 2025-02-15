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
    # ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    ros-humble-joy \
    libglfw3-dev \
    libglew-dev \
    libgps-dev \
    iproute2 net-tools \
    x11-apps \
    iputils-ping \
    # for opencv
     wget g++ unzip \ 
    && rm -rf /var/lib/apt/lists/*

# install opencv

RUN mkdir -p /home/opencv
RUN cd /home/opencv

RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
RUN unzip opencv.zip

RUN mkdir -p build && cd build
 
# Configure
RUN cmake  ../opencv-4.x
 
# Build
RUN cmake --build .

# Copy the urcAssets directory to the home directory in the container
RUN mkdir -p /home/urcAssets
COPY urcAssets /home/urcAssets

FROM ros:humble-ros-base-jammy AS plugin_installer

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-image-transport-plugins \
    ros-humble-xacro \
    ros-humble-moveit \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros-gz \
    ros-humble-ros-ign-bridge \
    ros-humble-ign-ros2-control \
    libqt5widgets5 \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-moveit-ros-planning-interface \
    tmux \
    ruby \
    vim nano \
    libboost-all-dev \
    libboost-dev \
    libboost-date-time-dev \
    libboost-filesystem-dev \
    libboost-program-options-dev libboost-system-dev libboost-thread-dev \
    && rm -rf /var/lib/apt/lists/*

FROM urc_software_base AS urc_software_builder

# Set the working directory
WORKDIR /ros2_ws

# Copy the libs
COPY ./libs /ros2_ws/libs

# Build pigpio from the submodule
RUN cd /ros2_ws/libs/pigpio && make && make install

# prepare to install all dependencies thorugh rosdep
# RUN cd /ros2_ws
# RUN rosdep update

COPY --from=plugin_installer /opt/ros/humble /opt/ros/humble
COPY --from=plugin_installer /usr/bin /usr/bin
COPY --from=plugin_installer /usr/share /usr/share
COPY --from=plugin_installer /usr/include /usr/include

# https://medium.com/codex/a-practical-guide-to-containerize-your-c-application-with-docker-50abb197f6d4
FROM urc_software_base AS urc_software 

# copy built binaries
WORKDIR /ros2_ws
COPY --from=plugin_installer /opt/ros/humble /opt/ros/humble
# for installing Qt for rviz2
COPY --from=plugin_installer /usr/lib /usr/lib
COPY --from=plugin_installer /usr/include /usr/include
# for installing gazebo
COPY --from=plugin_installer /usr/bin /usr/bin
COPY --from=plugin_installer /usr/share /usr/share
COPY ./install /ros2_ws/install
COPY ./libs /ros2_ws/libs
COPY ./run_nodes.sh /ros2_ws/run_nodes.sh

RUN cd /ros2_ws/libs/pigpio && make && make install

# Default command
ENTRYPOINT ["/ros2_ws/run_nodes.sh"]
#  ENTRYPOINT ["bash"]

# added
