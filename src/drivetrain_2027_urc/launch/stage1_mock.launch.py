"""Start only the software arbiter. No CAN backend or automatic mock feedback source."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="drivetrain_2027_urc", executable="dcs_stage1.py", output="screen"),
    ])
