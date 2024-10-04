# Use the official ROS 2 Humble base image
FROM ros:humble-ros-base-jammy

# Install ROS 2 core packages and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \    
    ros-humble-demo-nodes-cpp \
    && rm -rf /var/lib/apt/lists/*

# Source the ROS 2 setup in bash sessions
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set default command to bash
CMD ["bash"]
