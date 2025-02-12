from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare(package='sdp_bot').find('sdp_bot')
    default_model_path = os.path.join(pkg_share, 'urdf', 'all_bot.urdf')
    controller_param_path = os.path.join(pkg_share, 'config', 'controller.yaml')
    print(controller_param_path)

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        # Publish robot description first
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')])
            }],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
                controller_param_path
            ],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
        ),
    ])