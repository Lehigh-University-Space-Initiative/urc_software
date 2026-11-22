"""
Launch file for the browser interface on the base station (run mode "web_ui" in run_nodes.sh).

Starts:
    rosbridge_websocket  - lets the web page publish and subscribe to ROS topics (port 9090)
    web_video_server     - streams camera topics to the page as ordinary web video (port 8080)
    web_ui_server        - serves the page itself (port 8000)
    science_recorder     - saves Science snapshots when the page asks

Open http://<base-station-ip>:8000 from the Steam Deck or any laptop on the rover network.

Run it with:
    ros2 launch web_ui_urc web_ui_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Return the four nodes behind the web page (ROS calls this function by name)."""
    return LaunchDescription([
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             name='rosbridge_websocket', output='screen',
             parameters=[{'port': 9090}]),
        Node(package='web_video_server', executable='web_video_server',
             name='web_video_server', output='screen',
             parameters=[{'port': 8080}]),
        Node(package='web_ui_urc', executable='web_server', output='screen',
             parameters=[{'port': 8000}]),
        Node(package='web_ui_urc', executable='science_recorder', output='screen'),
    ])
