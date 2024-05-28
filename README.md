# To run inside the urc_software/src folder

```
docker build -t joy_mapper:humble .
docker build -t ros2_container .
docker run joy_mapper:humble
```

## To run a specific node, specify that node in the CMD directive in the Dockerfile. Example

```
CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 run base_station_urc gui_node"]
```

## Default command to run when starting the container

```
CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 launch base_station_urc base_station_launch.py"]
```
