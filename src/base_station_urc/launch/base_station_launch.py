from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node to handle joystick input 0
        Node(
            package='joy',
            executable='joy_node',
            name='joy0_node',
            parameters=[{'device_id': 0, 'coalesce_interval': 0.03}],
            remappings=[('/joy', '/joy0')],
            output='screen',
        ),
        # Node to handle joystick input 1
        Node(
            package='joy',
            executable='joy_node',
            name='joy1_node',
            parameters=[{'device_id': 1, 'coalesce_interval': 0.03}],
            remappings=[('/joy', '/joy1')],
            output='screen',
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
