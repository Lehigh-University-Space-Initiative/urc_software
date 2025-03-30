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
        # GroundStationGUI node
        Node(
            package='base_station_urc',
            executable='GroundStationGUI',
            name='ground_station_gui',
            output='screen',
            # parameters=[{'/hootl': False}] 
        ),
        Node(
            package='base_station_urc',
            executable='SpaceMouseMapper_node',
            name='space_mouse_mapper',
            output='screen',
            # parameters=[{'/hootl': False}] 
        ),
        Node(
            package='base_station_urc',
            executable='LUSIVisionStreamer_node',
            name='lusi_vision_streamer',
            output='screen',
            # parameters=[{'/hootl': False}] 
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='republish',
            output='screen',
            arguments=[
                'compressed',  # Input transport type
                '--ros-args',
                '--remap', 'in/compressed:=/video_stream/compressed',
                '--remap', 'out:=/video_stream/image_raw',
            ],
        ),
        # ros2 run image_transport republish compressed --ros-args --remap in/compressed:=camera/image_raw/compressed --remap out:=image
    ])
