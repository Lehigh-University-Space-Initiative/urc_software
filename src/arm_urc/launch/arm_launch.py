"""
Launch file for the arm Raspberry Pi (run mode "arm" in run_nodes.sh)

Starts:
    ArmMotorManager - drives the arm joints and gripper over CAN bus 1 (see src/main.cpp)

Run it with:
    ros2 launch arm_urc arm_launch.py
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
        A LaunchDescription containing the arm motor node
    """
    return LaunchDescription ([
        Node (
            package="arm_urc",
            executable="ArmMotorManager",
            output="screen",  # Print the node's logs to this terminal
            # Re-enabling the shared motor code's per-tick DEBUG timing logs (the driveline leaves them off)
            # Delete this line to quiet the arm's logs too
            arguments=["--ros-args", "--log-level", "MotorManager:=debug", "--log-level", "CANDriver:=debug"]
        )
    ])
