"""
Launch file for the rover's main computer (run modes "main_computer" and "hootl" in run_nodes.sh)

Starts:
    rviz2 - 3D view of the arm and its MoveIt planning scene
    move_group - MoveIt's planner, which also publishes the robot description (URDF) on /robot_description
    ros2_control_node - the controller manager; it loads the MockArmHardware plugin (the MoveIt-to-arm bridge)
    joint_state_broadcaster / arm_controller - ros2_control controllers, started by "spawner" helper nodes
    servo_node - MoveIt Servo, which turns SpaceMouse twist commands into smooth joint motion
    DriveTrainManager - turns /cmd_vel into per-wheel speeds for the driveline Pi
    VideoStreamer - publishes the selected rover camera
    video_compress - redundant image_transport relay (VideoStreamer already publishes the compressed stream; see below)

Arguments:
    gui_only - "true" starts only RViz and Servo (skips move_group, ros2_control, and the controllers)

Run it with:
    ros2 launch main_computer_urc main_computer_launch.py gui_only:=false
"""

# Imports

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder




# ---------------------------------
# Helpers
# ---------------------------------

def load_yaml(package_name, file_path):
    """
    Read a YAML file from an installed package's share directory

    Args:
        package_name: ROS package that installed the file (e.g. "main_computer_urc")
        file_path: path to the file relative to that package's share directory

    Returns:
        The parsed YAML as Python dicts/lists, or None if the file can't be opened
    """
    packagePath = get_package_share_directory(package_name)
    absoluteFilePath = os.path.join(packagePath, file_path)

    try:
        with open(absoluteFilePath, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # Parent of IOError and OSError
        return None




# ---------------------------------
# Launch description
# ---------------------------------

def generate_launch_description():
    """
    Build the list of nodes ROS starts for the main computer

    ROS calls this function by name when you run `ros2 launch`, so the name can't change

    Returns:
        A LaunchDescription with the MoveIt arm stack, the drive relay, and the video pipeline
    """

    """
    Launch arguments and file paths

    - A launch argument is a value you can set on the command line (e.g. gui_only:=true)
    - LaunchConfiguration("gui_only") reads that value later, when the launch file actually runs
    - FindPackageShare locates a package's installed share/ directory, where launch and config files live
    """
    declaredArguments = [
        DeclareLaunchArgument (
            "gui_only",
            default_value="false",
            description="Start only RViz and Servo, without move_group, ros2_control, or the controllers"
        )
    ]
    guiOnly = LaunchConfiguration("gui_only")

    rvizFile = PathJoinSubstitution([FindPackageShare("moveit_config_urc"), "config", "moveit.rviz"])

    robotControllers = PathJoinSubstitution([FindPackageShare("moveit_config_urc"), "config", "ros2_controllers.yaml"])

    """
    MoveIt configuration

    - MoveItConfigsBuilder gathers the URDF, SRDF, kinematics, joint limits, and planner settings
    - The name "gen3" is left over from MoveIt's Kinova Gen3 example; it only affects a default file lookup
    - The URDF actually comes from moveit_config_urc/.setup_assistant (main_computer_urc/description/robot.urdf.xacro)
    - The SRDF comes from the same file (moveit_config_urc/config/2dof_robot.srdf)
    """
    moveitConfig = (
        MoveItConfigsBuilder("gen3", package_name="moveit_config_urc")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor (
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    # MoveIt Servo settings live in this package's config/simulation_config.yaml, under the "moveit_servo" key
    servoYaml = load_yaml("main_computer_urc", "config/simulation_config.yaml")
    servoParams = {"moveit_servo": servoYaml}

    """
    Arm nodes

    - RViz needs the kinematics and planning parameters to show MoveIt's interactive markers (moveit2 issue #3339)
    - UnlessCondition(guiOnly) skips a node when gui_only:=true
    - ros2_control_node gets the robot description from the /robot_description topic that move_group publishes
    """
    rvizNode = Node (
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rvizFile],
        parameters=[
            moveitConfig.joint_limits,
            moveitConfig.robot_description_kinematics,
            moveitConfig.planning_pipelines
        ]
    )

    moveGroupNode = Node (
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveitConfig.to_dict()],
        condition=UnlessCondition(guiOnly)
    )

    controlNode = Node (
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robotControllers],
        output="both",
        remappings=[("~/robot_description", "/robot_description")],
        condition=UnlessCondition(guiOnly)
    )

    # A spawner is a short-lived helper that asks the controller manager to load and start one controller
    jointStateBroadcasterSpawner = Node (
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        condition=UnlessCondition(guiOnly)
    )

    armControllerSpawner = Node (
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        condition=UnlessCondition(guiOnly)
    )

    servoNode = Node (
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servoParams,
            moveitConfig.robot_description,
            moveitConfig.robot_description_semantic,
            moveitConfig.robot_description_kinematics
        ],
        output="screen"
    )

    """
    Drive and video nodes

    - DriveTrainManager and VideoStreamer are this package's own C++ nodes (see src/)
    - video_compress is an image_transport relay with input transport "compressed" and its topics remapped onto /video_stream
    - Verified in hootl: it subscribes to nothing and only adds a second publisher on /video_stream/compressed
    - VideoStreamer's own image_transport publisher already provides that topic, so this relay can likely be removed
    """
    driveTrainManagerNode = Node (
        package="main_computer_urc",
        executable="DriveTrainManager_node",
        name="DriveTrainManager",
        output="screen"
    )

    videoStreamerNode = Node (
        package="main_computer_urc",
        executable="VideoStreamer_node",
        name="VideoStreamer",  # The GUI's video panel sets this node's stream_cam parameter by this name
        output="screen"
    )

    videoRepublishNode = Node (
        package="image_transport",
        executable="republish",
        name="video_compress",  # Unique name: the base station runs its own relay, and hootl puts both on one machine
        output="screen",
        arguments=[
            "compressed",  # Input transport type
            "--ros-args",
            "--remap", "in:=/video_stream",
            "--remap", "out/compressed:=/video_stream/compressed"
        ]
    )

    # Intentionally not started: robot_state_publisher, joint_state_publisher_gui, Gazebo, and StatusLED_node
    # The argument declarations must come first, because launch evaluates actions in order
    # Each node's UnlessCondition reads gui_only, which fails ("launch configuration 'gui_only' does not exist") if it isn't declared yet
    # They used to be last, which broke hootl mode (it doesn't pass gui_only on the command line)
    return LaunchDescription (declaredArguments + [
        rvizNode,
        moveGroupNode,
        controlNode,
        jointStateBroadcasterSpawner,
        armControllerSpawner,
        servoNode,
        driveTrainManagerNode,
        videoStreamerNode,
        videoRepublishNode
    ])
