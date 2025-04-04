from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch_ros.substitutions import FindPackageShare 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


import os
import yaml

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui_only",
            default_value="false",
            description="",
        )
    )
    gui_only = LaunchConfiguration("gui_only")

    ## Setup for State publishing

    urdf_file = PathJoinSubstitution(
        [FindPackageShare("main_computer_urc"), "description", "robot.urdf.xacro"]
    )
    initial_file = PathJoinSubstitution(
        [FindPackageShare("moveit_config_urc"), "config", "initial_positions.yaml"]
        # [FindPackageShare("moveit_config_urc"), "config", "moveit.rviz"]
    )
    # rviz_file = PathJoinSubstitution(
    #     [FindPackageShare("main_computer_urc"), "description", "robot.rviz"]
    # )
    rviz_file = PathJoinSubstitution(
        [FindPackageShare("moveit_config_urc"), "config", "moveit.rviz"]
    )
    # ros2_control_config = PathJoinSubstitution(
    #     [FindPackageShare("main_computer_urc"), "description", "config", "ros2_control.yaml"]
    # )


    robot_description_content = Command([
        'xacro ',
        urdf_file,
    ])
    robot_description = {"robot_description": robot_description_content}    


    kinimatics_yaml = PathJoinSubstitution(
        [FindPackageShare("moveit_config_urc"), "config", "kinematics.yaml"]
    )
    joint_limits_yaml = PathJoinSubstitution(
        [FindPackageShare("moveit_config_urc"), "config", "kinematics.yaml"]
    )

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "gen3", package_name="moveit_config_urc"
        )
        # .robot_description(file_path=urdf_file, mappings={"initial_positions_file": "config/initial_positions.yaml"})
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        # .planning_pipelines(
        #     pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
        # )
        .to_moveit_configs()
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    #https://github.com/moveit/moveit2/issues/3339
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file],
        parameters = [
            moveit_config.joint_limits,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
        # condition=IfCondition(gui_only),
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("moveit_config_urc"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        # arguments = ["--ros-args", "--log-level", "debug"],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        condition=UnlessCondition(gui_only),
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        condition=UnlessCondition(gui_only),
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        condition=UnlessCondition(gui_only),
    )


    srdf_path = "/ros2_ws/install/share/moveit_config_urc/config/2dof_robot.srdf"

    with open(srdf_path, 'r') as f:
        semantic_content = f.read()

    # Get parameters for the Servo node
    servo_yaml = load_yaml("main_computer_urc", "config/simulation_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}


    move_group_node = Node(package='moveit_ros_move_group', executable='move_group',
                       output='screen',
                       parameters=[moveit_config.to_dict()],
                        condition=UnlessCondition(gui_only),
                       )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": " -r -v 3 empty.sdf"}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "rrbot_system_position",
            "-allow_renaming",
            "true",
        ],
    )


    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )



    ld = LaunchDescription([
        # robot_state_publisher_node,
        rviz_node,
        # # joint_state_publisher_node,
        # this env is for gazebo to work on M1 Mac
        # SetEnvironmentVariable(name="LIBGL_DRI3_DISABLE", value="1"),
        # gazebo,
        # gz_spawn_entity,
        move_group_node,
        control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        servo_node,
        Node(
            package='main_computer_urc',
            executable='DriveTrainManager_node',
            name='DriveTrainManager',
            output='screen'
        ),
        # Node(
        #     package='main_computer_urc',
        #     executable='StatusLED_node',
        #     name='StatusLED',
        #     output='screen'
        # ),
        Node(
            package='main_computer_urc',
            executable='VideoStreamer_node',
            name='VideoStreamer',
            output='screen'
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='republish',
            output='screen',
            arguments=[
                'compressed',  # Input transport type
                '--ros-args',
                '--remap', 'in:=/video_stream',
                '--remap', 'out/compressed:=/video_stream/compressed',
            ],
        ),
    ] + declared_arguments)

#     ld.add_action(
#     ExecuteProcess(
#         cmd=[[
#             FindExecutable(name='ros2'),
#             " service call ",
#             "/servo_node/start_servo ",
#             "std_srvs/srv/Trigger ",
#             '"{}"',
#         ]],
#         shell=True
#     )
# )

    return ld;

# <ros2_control name="${name}" type="system">
#             <hardware>
#                 <!-- By default, set up controllers for simulation. This won't work on real hardware -->
#                 <plugin>mock_components/GenericSystem</plugin>
#             </hardware>
#             <joint name="shoulder">
#                 <command_interface name="position"/>
#                 <state_interface name="position">
#                   <param name="initial_value">${initial_positions['shoulder']}</param>
#                 </state_interface>
#                 <state_interface name="velocity"/>
#             </joint>
#             <joint name="elbow">
#                 <command_interface name="position"/>
#                 <state_interface name="position">
#                   <param name="initial_value">${initial_positions['elbow']}</param>
#                 </state_interface>
#                 <state_interface name="velocity"/>
#             </joint>

#         </ros2_control>