"""
Launch file for GNSS waypoint following on the rover (uses the real /gps_data)

Starts:
    WaypointFollower - drives toward one GNSS waypoint by publishing /cmd_vel (see src/WaypointFollower/main.cpp)

Arguments:
    target_lat, target_lon - the waypoint in degrees
    arrival_radius_m - how close (in meters) counts as arrived

Run it with:
    ros2 launch navigation_urc navigation_launch.py target_lat:=38.4065 target_lon:=-110.7915

For a hardware-free test with a simulated GPS, use navigation_sim_launch.py instead
"""

# Imports

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue




# ---------------------------------
# Launch description
# ---------------------------------

def generate_launch_description():
    """
    Build the list of nodes ROS starts for this launch file

    ROS calls this function by name when you run `ros2 launch`, so the name can't change

    Returns:
        A LaunchDescription containing the waypoint follower and its arguments
    """
    arguments = [
        DeclareLaunchArgument("target_lat", default_value="0.0", description="Waypoint latitude in degrees"),
        DeclareLaunchArgument("target_lon", default_value="0.0", description="Waypoint longitude in degrees"),
        DeclareLaunchArgument("arrival_radius_m", default_value="3.0", description="Distance in meters that counts as arrived")
    ]

    # ParameterValue(..., value_type=float) turns the argument text into a real number parameter
    waypointFollowerNode = Node (
        package="navigation_urc",
        executable="WaypointFollower_node",
        name="WaypointFollower",
        output="screen",
        parameters=[{
            "target_lat": ParameterValue(LaunchConfiguration("target_lat"), value_type=float),
            "target_lon": ParameterValue(LaunchConfiguration("target_lon"), value_type=float),
            "arrival_radius_m": ParameterValue(LaunchConfiguration("arrival_radius_m"), value_type=float)
        }]
    )

    return LaunchDescription(arguments + [waypointFollowerNode])
