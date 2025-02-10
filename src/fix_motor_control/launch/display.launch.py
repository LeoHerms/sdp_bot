from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_path = get_package_share_directory('fix_motor_control')
    urdf_file = os.path.join(package_path, 'urdf', 'sdp_bot.urdf')

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Use simulation time if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file]),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(package_path, 'rviz', 'display.rviz')]
        )
    ])
