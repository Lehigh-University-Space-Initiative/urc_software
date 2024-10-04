# ROS2 multi computer communication


Resource: [ROS2 Multiple Machines](https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/#Use_ROS_DOMAIN_ID_to_run_multiple_separate_ROS2_applications_on_the_same_network)

## Testing

### Container 1:


```bash
docker build -t container1 .

docker run -it --rm --name ros2-node1 -p 7400:7400/udp -p 7500:7500/udp container1

source /opt/ros/humble/setup.bash

ros2 run demo_nodes_cpp talker
```

### Container 2:

```bash
docker build -t container2 .

docker run -it --rm --name ros2-node2 -p 7401:7400/udp -p 7501:7500/udp container2    

source /opt/ros/humble/setup.bash

ros2 run demo_nodes_cpp listener
```