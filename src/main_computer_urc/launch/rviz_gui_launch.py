"""
Launch file that starts only RViz with the arm's MoveIt view (run mode "rviz" in run_nodes.sh)

Useful on a laptop to watch the arm while the main computer runs the real stack

Run it with:
    ros2 launch main_computer_urc rviz_gui_launch.py
"""

# Imports

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare




# ---------------------------------
# Launch description
# ---------------------------------

def generate_launch_description():
    """
    Build the list of nodes ROS starts for this launch file

    Returns:
        A LaunchDescription containing a single RViz node using moveit_config_urc's saved view
    """
    rvizFile = PathJoinSubstitution([FindPackageShare("moveit_config_urc"), "config", "moveit.rviz"])

    rvizNode = Node (
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rvizFile]  # -d loads a saved RViz display configuration
    )

    return LaunchDescription([rvizNode])
