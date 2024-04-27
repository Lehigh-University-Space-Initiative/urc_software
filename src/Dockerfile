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
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Install build tools, libraries for ImGui, GLFW, and GLEW
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libglfw3-dev \
    libglew-dev

# Clone ImGui and build it with the GLFW and OpenGL3 examples
WORKDIR /tmp
RUN git clone https://github.com/ocornut/imgui.git && \
    cd imgui && \
    cd examples/libs/gl3w && \
    ./GL3W_gen.py && \
    cd ../../ && \
    g++ -c -I./ -Iexamples/libs/gl3w examples/libs/gl3w/GL/gl3w.c && \
    g++ -c -I./ -Iexamples/libs/gl3w examples/example_glfw_opengl3/main.cpp examples/imgui_impl_glfw.cpp examples/imgui_impl_opengl3.cpp && \
    g++ -o imgui_demo *.o -lglfw -ldl -lGL && \
    cp imgui_demo /usr/local/bin/

# Remove unnecessary packages and clean up
RUN apt-get remove --purge -y git build-essential cmake && \
    apt-get autoremove -y && \
    rm -rf /var/lib/apt/lists/* /tmp/imgui

# Copy ROS 2 workspace
WORKDIR /ros2_ws
COPY ./base_station_urc /ros2_ws/src/base_station_urc

# Build the ROS 2 workspace
ENV ImGui_DIR=/path/to/imgui/cmake
RUN /bin/bash -c '. /opt/ros/humble/setup.sh && colcon build --symlink-install'

# Source the workspace
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command to run when starting the container
CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 run base_station_urc gui_node"]
