# To run from the root of the repository

```bash
docker build -t ros_base_station:latest .
docker run -it ros_base_station:latest /bin/bash
ros2 launch base_station_urc base_station_launch.py
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
xcalc # to test if GUI is working
ros2 launch base_station_urc base_station_launch.py
```

## Setting up GUI forwarding on Windows

- Go [here](https://sourceforge.net/projects/vcxsrv/) to install VcXsrv
- Launch it and choose the following settings
  - Multiple windows
  - Start no client
  - Uncheck “Native opengl” and check “Disable access control”
- When executing VcXsrv for the first time, a Windows Firewall pop-up should appear. Just be sure to give it permissions on both private and public networks.
- If the pop-up doesn't appear, or if you gave it the wrong permissions by mistake, you can just change the permissions manually. To do so, open Windows Security app and go to Firewall & network protection -> Allow an app through firewall and ensure VcXsrv windows x server has both permissions.
- We need to set the DISPLAY environment variable which tells WSL2 where to send the graphics for any application that needs a display to function. To do this, you’ll need to know the IP address for the Windows host machine. You can find this by going to Settings -> Network & Internet and looking under the properties of your current connection (it will probably be labeled as “IPv4 Address”)
- Go to [Dockerfile](Dockerfile) and replace `{your_ip_address}` with your IP address in this line
  - `RUN echo 'export DISPLAY={your_ip_address}:0.0' >> ~/.bashrc`
  - For example, this is mine: `RUN echo 'export DISPLAY=10.0.0.155:0.0' >> ~/.bashrc`
- Build the docker image and run it as shown above

Credits to this [article](https://jackkawell.wordpress.com/2020/06/12/ros-wsl2/>)
and this [article](https://aalonso.dev/blog/2021/how-to-use-gui-apps-in-wsl2-forwarding-x-server-cdj)

## GUI Forwarding on Linux

```
export DISPLAY=:0.0
xhost +local:docker
sudo docker run -it -e DISPLAY=$DISPLAY --net=host lusi_software
```

## To run a specific node, specify that node in the CMD directive in the Dockerfile. Example

```bash
CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 run base_station_urc gui_node"]
```

## Default command to run when starting the container

```bash
CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 launch base_station_urc base_station_launch.py"]
```
