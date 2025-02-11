from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess

from ros_gz_sim.actions import GzServer
from ros_gz_sim.actions import GzSpawnModel
from ros_gz_bridge.actions import RosGzBridge

import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='sdp_bot').find('sdp_bot')
    gazebo_pkg_share = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    print(gazebo_pkg_share)
    default_model_path = os.path.join(pkg_share, 'urdf', 'gz_sdp_bot.sdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    world_path = os.path.join(pkg_share, 'world', 'my_world.sdf')

    gz_spawn_model_launch_source = os.path.join(gazebo_pkg_share, 'launch', 'gz_spawn_model.launch.py')
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'my_world',
            'topic': '/robot_description',
            'entity_name': 'sdp_bot',
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
        gz_server,
        ros_gz_bridge,
        spawn_entity,
        robot_state_publisher_node,
        rviz_node
    ])