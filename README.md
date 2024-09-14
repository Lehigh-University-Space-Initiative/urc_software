# To run from the root of the repository

## On Windows

```bash
docker build -t urc_software .
export DISPLAY=128.180.198.214:0.0 # replace with your IP address up to the colon
docker run -it -e DISPLAY=$DISPLAY --net=host urc_software
docker run --rm -v C:/Users/phamd/urc_software:/root --name urc_software_dev -it -e DISPLAY=$DISPLAY --net=host urc_software
```

## Debugging inside docker

```bash
docker run -it urc_software /bin/bash
```

Verify executables

```bash
ros2 pkg executables main_computer_urc
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
