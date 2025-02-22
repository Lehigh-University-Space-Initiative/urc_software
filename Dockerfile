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
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
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

RUN apt update
RUN apt -y dist-upgrade
RUN colcon mixin update default
RUN mkdir -p /ros2_ws/src
RUN cd /ros2_ws/src
RUN git clone --branch humble https://github.com/ros-planning/moveit2_tutorials
RUN vcs import < moveit2_tutorials/moveit2_tutorials.repos
RUN apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
RUN cd /ros2_ws/
RUN bash -c 'source /opt/ros/humble/setup.bash && colcon build --mixin release --merge-install --packages-skip imgui_example_glfw_wgpu ImGuiExample imgui_example_glfw_vulkan pigpio'
RUN bash -c 'source /ros2_ws/install/setup.bash'
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-servo \
    && rm -rf /var/lib/apt/lists/*

# https://medium.com/codex/a-practical-guide-to-containerize-your-c-application-with-docker-50abb197f6d4
FROM urc_software_base AS urc_software 

# copy built binaries
WORKDIR /ros2_ws
COPY --from=plugin_installer /opt/ros/humble /opt/ros/humble
COPY --from=urc_software_builder /opt/ros/humble /opt/ros/humble
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
