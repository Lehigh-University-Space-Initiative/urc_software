# Multi-stage build of the urc_software image (one image runs on every rover computer and the base station)
#
# Stages:
#   urc_software_base     - Ubuntu 22.04 + ROS 2 Humble + every apt dependency + sockpp (shared by the other two)
#   urc_software_builder  - base + pigpio; softwareUpdate/dockerBuild.sh runs colcon inside this with the repo mounted
#   urc_software          - base + the compiled install/ folder from the host; this is the image you run
#
# The code itself is compiled outside "docker build", by dockerBuild.sh with the repo mounted as /ros2_ws
# That keeps build/ on the host so rebuilds are incremental (pattern from https://github.com/pcewing/docker-incremental-compile-demo)
#
# Caching note: Docker reuses a step's cached result only if its instruction text is unchanged
# Editing a RUN line (even whitespace) forces that step and everything after it to rebuild, which takes a long time here

FROM ros:humble-ros-base-jammy AS urc_software_base

# Installing ROS 2 packages, build tools, GUI libraries (GLFW/GLEW), and debugging tools
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

# Copying runtime image assets (e.g. the GUI's "no downlink" placeholder) to /home/urcAssets
RUN mkdir -p /home/urcAssets
COPY urcAssets /home/urcAssets

# Building and installing sockpp (TCP sockets for LUSIVisionStreamer) from the libs/ submodule into /usr/local
COPY ./libs /ros2_ws/libs
RUN cd /ros2_ws/libs/sockpp && cmake -Bbuild . && cmake --build build/ && cmake --build build/ --target install

FROM urc_software_base AS urc_software_builder

WORKDIR /ros2_ws

COPY ./libs /ros2_ws/libs

# Building and installing pigpio (Raspberry Pi GPIO) into /usr/local; arm_urc and driveline_urc link this copy
RUN cd /ros2_ws/libs/pigpio && make && make install

# Final runtime image (pattern from https://medium.com/codex/a-practical-guide-to-containerize-your-c-application-with-docker-50abb197f6d4)
FROM urc_software_base AS urc_software 

WORKDIR /ros2_ws
COPY --from=urc_software_builder /opt/ros/humble /opt/ros/humble
# Copying the colcon output from the host; install/ only exists after dockerBuild.sh's colcon step has run
COPY ./install /ros2_ws/install
COPY ./libs /ros2_ws/libs
COPY ./run_nodes.sh /ros2_ws/run_nodes.sh

RUN cd /ros2_ws/libs/pigpio && make && make install

# Every `docker run ... urc_software <mode>` runs run_nodes.sh <mode>; use the "manual" mode for a shell
ENTRYPOINT ["/ros2_ws/run_nodes.sh"]
