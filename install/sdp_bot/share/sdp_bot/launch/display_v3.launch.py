from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('sdp_bot')
    
    # URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'all_bot.urdf')
    
    # RViz config file
    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )
    
    # Control node
    control_node = Node(
        package='sdp_bot',
        executable='control',
        name='motor_controller',
        output='screen'
    )
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        control_node,
        rviz_node
    ])