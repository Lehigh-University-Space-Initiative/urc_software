"""
Launch file for the driveline Raspberry Pi (run modes "driveline" and "hootl" in run_nodes.sh)

Starts:
    MotorController - MotorCtr_node, which drives the six wheel motors over CAN bus 0 (see src/main.cpp)

Run it with:
    ros2 launch driveline_urc driveline_launch.py
"""

# Imports

from launch import LaunchDescription
from launch_ros.actions import Node




# ---------------------------------
# Launch description
# ---------------------------------

def generate_launch_description():
    """
    Build the list of nodes ROS starts for this launch file

    ROS calls this function by name when you run `ros2 launch`, so the name can't change

    Returns:
        A LaunchDescription containing the wheel motor node
    """
    return LaunchDescription ([
        Node (
            package="driveline_urc",
            executable="MotorCtr_node",  # Executable name from add_executable() in CMakeLists.txt
            name="MotorController",
            output="screen"  # Print the node's logs to this terminal
        )
    ])
