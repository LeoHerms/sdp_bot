from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('sdp_bot')
    
    # Define paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'all_bot.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'sensor_test.rviz')
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )
    
    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    
    # IMU node
    imu_node = Node(
        package='sdp_bot',
        executable='bno055',
        name='imu_sensor',
        parameters=[{
            'frame_id': 'imu_link'
        }]
    )
    
    # RPLidar node
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'lidar_link',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
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
        joint_state_publisher_gui,
        imu_node,
        lidar_node,
        rviz_node
    ])