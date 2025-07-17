# AutoFactory Robot Navigation System

## Introduction

AutoFactory aims to replicate the scalability of autonomous systems with a simple robot navigation implementation. This project demonstrates autonomous robot navigation using ROS2 (Jazzy Jalisco) with distributed computation between a Raspberry Pi 5 and a main control machine.

## Hardware Components

### Core Platform
- **Raspbot Base Frame**: Motor interface through PCB frame linking to Raspberry Pi 5
  - Motors communicate via I2C protocol
  - Provides stable 3-layer mounting platform

### Sensors & Controllers
- **RPLidar A1M8**: Positioned at highest point for obstacle detection
  - USB serial communication to Raspberry Pi 5
  - 360° laser scanning capability

- **BNO055 IMU Module**: Inertial Measurement Unit for robot localization
  - I2C communication to Arduino Pro Micro
  - Provides orientation and acceleration data

- **Arduino Pro Micro**: Sensor data relay platform
  - Processes IMU readings
  - USB serial communication to Raspberry Pi 5

- **Raspberry Pi 5**: Primary sensor data hub
  - Runs ROS2 publisher nodes
  - Publishes to `/imu`, `/scan`, and `/odom` topics

## System Architecture

The system uses a distributed computing approach:
- **Raspberry Pi 5**: Handles sensor data collection and publishing
- **Main Machine**: Performs heavy navigation computations (SLAM, path planning, localization)

## Prerequisites

### Operating System
- Ubuntu 24.04 on both Raspberry Pi 5 and main machine
- ROS2 "Jazzy Jalisco" fully installed and sourced

### Required Packages

#### On Raspberry Pi 5:
```bash
# Update system
sudo apt update && sudo apt upgrade

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Install Nav2
sudo apt install ros-jazzy-rplidar-ros
```

#### On Main Machine:
```bash
# Install robot localization
sudo apt install ros-jazzy-robot-localization

# Install Flask for API functionality
sudo apt-get install python3-flask python3-flask-cors
```

## Setup Instructions

### 1. Hardware Assembly
Assemble robot in a 3-layer stack:
- **Bottom layer**: Raspberry Pi 5
- **Middle layer**: IMU module  
- **Top layer**: RPLidar unit

### 2. RPLidar Setup
```bash
# Verify USB connection
ls /dev/ttyUSB*
# Should show /dev/ttyUSB0

# If USB not recognized, grant permissions:
sudo usermod -aG dialout $USER
# Restart system after this command

# Test RPLidar
ros2 launch rplidar_ros view_rplidar.launch.py
```

### 3. Workspace Setup
```bash
# Pull workspace and build packages
cd your_workspace
colcon build
source install/setup.bash
```

## Usage

### Step 1: Start Sensor Data Collection (Raspberry Pi 5)
```bash
# Source workspace
source install/setup.bash

# Launch sensor publishers
ros2 launch sdp_bot sdp.launch.py
```

### Step 2: Mapping Phase (Main Machine)

For initial environment mapping:

```bash
# Terminal 1: Extended Kalman Filter
ros2 launch sdp_bot ekf.launch.py

# Terminal 2: SLAM for mapping
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false

# Terminal 3: Teleoperation (Raspberry Pi 5)
ros2 launch sdp_bot teleop.launch.py

# Terminal 4: Manual control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 5: Visualization
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### Step 3: Autonomous Navigation (Main Machine)

Execute in order:

```bash
# Terminal 1: Extended Kalman Filter
ros2 launch sdp_bot ekf.launch.py

# Terminal 2: Localization
ros2 launch nav2_bringup localization_launch.py \
  map:=my_room.yaml \
  params_file:=Desktop/sdp_bot/src/sdp_bot/config/min_params.yaml \
  use_sim_time:=false

# Terminal 3: Set initial pose
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --once

# Terminal 4: Navigation stack
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=Desktop/sdp_bot/src/sdp_bot/config/min_params.yaml \
  use_sim_time:=false

# Terminal 5: Visualization
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### Step 4: Web Interface (Optional)
```bash
# Navigate to workspace and source
cd your_workspace
source install/setup.bash

# Navigate to package source and run Flask client
cd src/sdp_bot
python3 nav2_flask_client.py
```

## Navigation Stack Components

The main machine handles:

### Sensor Fusion
- **Extended Kalman Filter**: Fuses wheel odometry and IMU readings
- **AMCL**: Adaptive Monte Carlo localization for mapping and localization

### Navigation Servers
- **Controller Server**: Executes planned routes with velocity commands
- **Smoother Server**: Refines paths from planner server
- **Planner Server**: Computes optimal paths to goals
- **Behavior Server**: Handles pre-defined movements (backing up, etc.)
- **Velocity Smoother**: Ensures smooth acceleration/deceleration
- **Collision Monitor**: Monitors for potential collisions
- **Behavior Tree Navigator**: Sends high-level commands like NavigateToPose
- **Waypoint Follower**: Enables following pre-defined waypoints

## Transform Tree Structure

The system requires these critical transforms:
- `map → odom`: Mapping and localization (AMCL/SLAM)
- `odom → base_footprint`: Sensor readings relative positioning  
- `base_footprint → base_link`: Static component positions (frame, wheels, lidar)

## Troubleshooting

### Common Issues
1. **USB Permission Errors**: Run `sudo usermod -aG dialout $USER` and restart
2. **Transform Errors**: Ensure all launch files are running in correct order
3. **Map Loading Issues**: Verify file paths in launch commands match your directory structure

### Verification Commands
```bash
# Check active topics
ros2 topic list

# Monitor sensor data
ros2 topic echo /scan
ros2 topic echo /imu
ros2 topic echo /odom

# Check transform tree
ros2 run tf2_tools view_frames
```

## References

- [Raspbot Documentation](http://www.yahboom.net/study/Raspbot)
- [Nav2 Documentation](https://docs.nav2.org/getting_started/index.html)
- [SLAM Toolbox Tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)
- [ROS2 Control Documentation](https://control.ros.org/rolling/index.html)

## Future Improvements

- [ ] Web interface integration with HTTPS functionality
- [ ] Hardware improvements (bolted connections, soldered electronics)
- [ ] Live camera streaming capability
- [ ] Enhanced parameter tuning for specific environments
- [ ] Comprehensive test case documentation

## Contributing

This project is part of ongoing robotics research. For questions or contributions, please refer to the documentation and ensure all dependencies are properly installed before reporting issues.
