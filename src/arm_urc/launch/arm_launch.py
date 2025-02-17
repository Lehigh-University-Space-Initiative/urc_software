import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('arm_urc'),
        'urdf',
        'arm_urc.urdf'
    )

    pwm_reader_node = Node(
            package='arm_urc',
            executable='arm_urc_pwm',
            output='screen'
        )
    
    # arm_manager_node = Node(
    #         package='arm_urc',
    #         executable='ArmMotorManager',
    #         output='screen',
    #     )
    
    return LaunchDescription([
        pwm_reader_node,
        # arm_manager_node,
    ])