# To run inside the urc_software/src folder

```
docker build -t joy_mapper:humble .
docker run joy_mapper:humble
```

## Debugging inside docker

```
docker run -it joy_mapper:humble /bin/bash
colcon build --symlink-install --packages-select cross_pkg_messages
source install/setup.bash
colcon build --symlink-install --packages-select base_station_urc
ls -la /ros2_ws/src/base_station_urc/include/cs_libguarded
ls install/base_station_urc/lib/base_station_urc
ros2 launch base_station_urc base_station_launch.py
```

```
docker build -t ros_base_station:latest .
docker run -it ros_base_station:latest /bin/bash
ls -la /ros2_ws/src/base_station_urc/include/cs_libguarded
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select cross_pkg_messages
source install/setup.bash
colcon build --symlink-install --packages-select base_station_urc
ros2 launch base_station_urc base_station_launch.py
```

## To run a specific node, specify that node in the CMD directive in the Dockerfile. Example

```
CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 run base_station_urc gui_node"]
```

## Default command to run when starting the container

```
CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 launch base_station_urc base_station_launch.py"]
```
