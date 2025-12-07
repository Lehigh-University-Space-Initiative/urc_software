from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import os

def generate_launch_description():
    robot_xacro = LaunchConfiguration('robot_xacro')
    controllers_yaml = LaunchConfiguration('controllers_yaml')

    #TODO: check this
    xacro_arg = DeclareLaunchArgument('robot_xacro', default_value=os.path.join(
        '/opt/ros/share/drive_hardware/urdf', 'robot.urdf.xacro'
    ))
    yaml_arg = DeclareLaunchArgument('controllers_yaml', default_value=os.path.join(
        '/opt/ros/share/drive_hardware/config', 'drive_power_forward.yaml' # likely need to switch this when changing between power and velocity
    ))

    state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', robot_xacro])}],
        output='screen'
    )

    #TODO: rename this?
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': Command(['xacro ', robot_xacro])}, controllers_yaml],
        output='screen'
    )

    #TODO: rename this?
    spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['drive_power'], #switch to 'diff_drive' for velocity control, 'drive_power' for power control
        output='screen'
    )

    return LaunchDescription([
        xacro_arg,
        yaml_arg,

        state_pub_node,
        ros2_control_node,
        spawner_node,
    ])


    # return LaunchDescription([
    #     Node(
    #         package='driveline_urc',
    #         executable='MotorCtr_node',
    #         name='MotorController',
    #         output='screen'
    #     )
    # ])
    