"""
Launch file for the Science Raspberry Pi (run mode "science" in run_nodes.sh).

Starts:
    science_node        - auger, platform, soil dropper, and chamber lid motors
    soil_probe_node     - 7-in-1 soil probe over RS-485
    weight_sensor_node  - HX711 load cell
    spectrometer_node   - USB camera spectrometer (raw intensity per pixel)
    camera_node         - science camera (raw and JPEG-compressed images)

Run it with:
    ros2 launch science_urc science_launch.py

Each node runs as its own process, so one missing device (for example an unplugged
spectrometer) stops only that node; the others keep running.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def science_node(executable):
    """
    Build a Node action for one science_urc executable.

    Args:
        executable: console_scripts name from setup.py.

    Returns:
        launch_ros Node action that prints its logs to this terminal.
    """
    return Node(package='science_urc', executable=executable, output='screen')


def generate_launch_description():
    """Return the five Science Module nodes (ROS calls this function by name)."""
    return LaunchDescription([
        science_node('science_node'),
        science_node('soil_probe_node'),
        science_node('weight_sensor_node'),
        science_node('spectrometer_node'),
        science_node('camera_node'),
    ])
