"""
Launch file for the base station laptop (run modes "base_station" and "hootl" in run_nodes.sh)

Starts:
    joy0_node, joy1_node - drivers for the two joysticks (ROS's standard joy package)
    joy_mapper - JoyMapper: joysticks to /cmd_vel
    ground_station_gui - GroundStationGUI: the operator window
    space_mouse_mapper - SpaceMouseMapper: SpaceMouse to arm commands
    lusi_vision_streamer - LUSIVisionStreamer: video and telemetry for LUSI Vision clients
    video_decompress - decompresses the rover's /video_stream/compressed into /video_stream/image_raw

Arguments:
    hootl - "true" for hardware-out-of-the-loop testing (the GUI shows a banner and treats rover links as connected)

Run it with:
    ros2 launch base_station_urc base_station_launch.py hootl:=false
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
    Build the list of nodes ROS starts on the base station

    ROS calls this function by name when you run `ros2 launch`, so the name can't change

    Returns:
        A LaunchDescription with the input, GUI, and video nodes
    """
    hootlArgument = DeclareLaunchArgument (
        "hootl",
        default_value="false",
        description="Hardware-out-of-the-loop test mode for the GUI and LUSI Vision telemetry"
    )
    # ParameterValue(..., value_type=bool) turns the "true"/"false" text into a real bool parameter
    hootlParam = ParameterValue(LaunchConfiguration("hootl"), value_type=bool)

    """
    Joystick drivers

    - device_id picks which connected joystick each driver reads (0 = first, 1 = second)
    - coalesce_interval_ms 30 batches rapid stick changes into at most one message every 30 ms
    - It was previously set as "coalesce_interval: 0.03" (the ROS 1 name), which Humble's joy_node silently ignored
    - The remap renames each driver's output topic from /joy to /joy0 or /joy1 so JoyMapper can tell them apart
    """
    joy0Node = Node (
        package="joy",
        executable="joy_node",
        name="joy0_node",
        parameters=[{"device_id": 0, "coalesce_interval_ms": 30}],
        remappings=[("/joy", "/joy0")],
        output="screen"
    )

    joy1Node = Node (
        package="joy",
        executable="joy_node",
        name="joy1_node",
        parameters=[{"device_id": 1, "coalesce_interval_ms": 30}],
        remappings=[("/joy", "/joy1")],
        output="screen"
    )

    """
    This package's own nodes

    - Executable names come from add_executable() in CMakeLists.txt
    - The GUI and LUSIVisionStreamer both read the hootl parameter
    """
    joyMapperNode = Node (
        package="base_station_urc",
        executable="JoyMapper_node",
        name="joy_mapper",  # The GUI's Telemetry panel sets swap_joysticks on the node by this name
        output="screen"
    )

    groundStationGuiNode = Node (
        package="base_station_urc",
        executable="GroundStationGUI",
        name="ground_station_gui",
        output="screen",
        parameters=[{"hootl": hootlParam}]
    )

    spaceMouseMapperNode = Node (
        package="base_station_urc",
        executable="SpaceMouseMapper_node",
        name="space_mouse_mapper",
        output="screen"
    )

    lusiVisionStreamerNode = Node (
        package="base_station_urc",
        executable="LUSIVisionStreamer_node",
        name="lusi_vision_streamer",
        output="screen",
        parameters=[{"hootl": hootlParam}]
    )

    # Same as: ros2 run image_transport republish compressed --ros-args --remap in/compressed:=... --remap out:=...
    videoRepublishNode = Node (
        package="image_transport",
        executable="republish",
        name="video_decompress",  # Unique name: the main computer runs its own relay, and hootl puts both on one machine
        output="screen",
        arguments=[
            "compressed",  # Input transport type (JPEG frames from the rover)
            "--ros-args",
            "--remap", "in/compressed:=/video_stream/compressed",
            "--remap", "out:=/video_stream/image_raw"
        ]
    )

    return LaunchDescription ([
        hootlArgument,
        joy0Node,
        joy1Node,
        joyMapperNode,
        groundStationGuiNode,
        spaceMouseMapperNode,
        lusiVisionStreamerNode,
        videoRepublishNode
    ])
