# To run inside the urc_software/src folder

```bash
docker build -t ros_base_station:latest .
docker run ros_base_station:latest
```

## Debugging inside docker

```bash
docker build -t ros_base_station:latest .
docker run -it ros_base_station:latest /bin/bash
ls -la /ros2_ws/src/base_station_urc/include/cs_libguarded
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select cross_pkg_messages
colcon build --symlink-install --packages-select base_station_urc
ros2 launch base_station_urc base_station_launch.py
```

## To run a specific node, specify that node in the CMD directive in the Dockerfile. Example

```bash
CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 run base_station_urc gui_node"]
```

## Default command to run when starting the container

```bash
CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 launch base_station_urc base_station_launch.py"]
```
