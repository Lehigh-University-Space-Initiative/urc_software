from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node to handle joystick input 0
        Node(
            package='joy',
            executable='joy_node',
            name='joy0_node',
            parameters=[{'dev': '/dev/input/js1', 'coalesce_interval': 0.3}],
            remappings=[('/joy', '/joy0')]
        ),
        # Node to handle joystick input 1
        Node(
            package='joy',
            executable='joy_node',
            name='joy1_node',
            parameters=[{'dev': '/dev/input/js2', 'coalesce_interval': 0.3}],
            remappings=[('/joy', '/joy1')]
        ),
        # JoyMapper node to process joystick inputs and send commands
        Node(
            package='base_station_urc',
            executable='JoyMapper_node',
            name='joy_mapper',
            output='screen',
        ),
        # Node(
        #     package='base_station_urc',
        #     executable='TelemetryPanel_node',
        #     name='telemetry_panel',
        #     output='screen',
        # ),
        # GroundStationGUI node
        Node(
            package='base_station_urc',
            executable='GroundStationGUI',
            name='ground_station_gui',
            output='screen',
            # parameters=[{'/hootl': False}] 
        ),
    ])
