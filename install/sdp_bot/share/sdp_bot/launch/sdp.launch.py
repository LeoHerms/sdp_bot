from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

# This launches the robot state publisher, IMU, RPLidar, and control nodes
# Should be done on the Raspberry Pi

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('sdp_bot')
    
    # Define paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'all_bot.urdf')
    
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
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
    
    # Control node
    control_node = Node(
        package='sdp_bot',
        executable='control',
        name='motor_controller',
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Flag to enable use_sim_time'),
        use_sim_time,
        robot_state_publisher,
        imu_node,
        lidar_node,
        control_node
    ])