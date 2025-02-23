from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

# Move the nav2 and lifecycle stuff to another launch file (to run on powerful laptop)

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('sdp_bot')
    
    # Define paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'all_bot.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'config_v2.rviz')
    nav2_config = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
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
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Robot localization node
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), 
                   {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Nav2 Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_config],
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    # Nav2 Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config]
    )

    # Nav2 Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[nav2_config],
        output='screen'
    )

    # Nav2 BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config]
    )

    # Nav2 Lifecycle Manager
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': ['controller_server',
                           'planner_server',
                           'behavior_server',
                           'bt_navigator']}]
    )

    # Create and return launch description
    return LaunchDescription([
        # Parameters first
        use_sim_time,
        
        # Core nodes
        robot_state_publisher,
        
        # Sensors
        imu_node,
        lidar_node,
        
        # Robot control and localization
        control_node,
        robot_localization_node,
        
        # Navigation stack
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,
        
        # Visualization last
        rviz_node,
    ])