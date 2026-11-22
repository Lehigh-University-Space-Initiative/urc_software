# Building only the simulation package, independently of the rover's cached build/install
FROM urc_software_builder:latest
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*
RUN useradd --create-home --uid 1000 --shell /bin/bash lusi_sim \
    && mkdir /sim_ws && chown lusi_sim:lusi_sim /sim_ws
WORKDIR /sim_ws
COPY --chown=lusi_sim:lusi_sim . src/rover_sim_urc/
USER lusi_sim
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select rover_sim_urc
CMD ["bash", "-c", "source install/setup.bash && ros2 launch rover_sim_urc assignment1.launch.py"]
