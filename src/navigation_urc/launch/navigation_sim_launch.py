"""
Hardware-free navigation simulation (run mode "nav_sim" in run_nodes.sh)

Starts:
    FakeGPS - simulated rover: integrates /cmd_vel into a position and publishes it on /gps_data
    WaypointFollower - drives toward the target by publishing /cmd_vel

The two nodes close the loop with each other, so you can watch the rover "drive" to the target in the logs:
    - WaypointFollower prints distance, bearing, heading, and its command once a second
    - "distance" should shrink until it prints "Arrived at target"

Arguments:
    start_lat, start_lon - simulated starting position (default: Mars Desert Research Station, Hanksville UT)
    target_lat, target_lon - the waypoint (default: about 21 m northeast of the start)
    arrival_radius_m - how close (in meters) counts as arrived

Run it with:
    ros2 launch navigation_urc navigation_sim_launch.py
    ros2 launch navigation_urc navigation_sim_launch.py target_lat:=38.4070 target_lon:=-110.7910
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
    Build the fake-GPS + waypoint-follower simulation

    Returns:
        A LaunchDescription containing both nodes and their arguments
    """
    arguments = [
        DeclareLaunchArgument("start_lat", default_value="38.4061", description="Simulated start latitude in degrees"),
        DeclareLaunchArgument("start_lon", default_value="-110.7918", description="Simulated start longitude in degrees"),
        DeclareLaunchArgument("target_lat", default_value="38.40625", description="Waypoint latitude in degrees"),
        DeclareLaunchArgument("target_lon", default_value="-110.79165", description="Waypoint longitude in degrees"),
        DeclareLaunchArgument("arrival_radius_m", default_value="3.0", description="Distance in meters that counts as arrived")
    ]

    # ParameterValue(..., value_type=float) turns the argument text into a real number parameter
    fakeGpsNode = Node (
        package="navigation_urc",
        executable="fake_gps_node.py",  # Installed by install(PROGRAMS ...) in CMakeLists.txt
        name="FakeGPS",
        output="screen",
        parameters=[{
            "start_lat": ParameterValue(LaunchConfiguration("start_lat"), value_type=float),
            "start_lon": ParameterValue(LaunchConfiguration("start_lon"), value_type=float)
        }]
    )

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

    return LaunchDescription(arguments + [fakeGpsNode, waypointFollowerNode])
