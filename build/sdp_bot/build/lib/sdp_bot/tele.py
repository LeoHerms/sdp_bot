#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf2_ros
from math import sin, cos
import smbus
import time

# WARNING: Need to remove transform from odom to base_link,
# as it will be published by the robot_localization node (or at least I think so)

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Robot parameters
        self.wheel_radius = 0.03175  # meters
        self.wheel_separation = 0.127  # meters
        self.wheel_names = [
            'base_front_left_wheel_joint',
            'base_front_right_wheel_joint',
            'base_back_left_wheel_joint',
            'base_back_right_wheel_joint'
        ]
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR
        self.last_update_time = self.get_clock().now()
        
        # Current velocity state
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        # Initialize I2C Bus
        self._addr = 0x16
        self._device = smbus.SMBus(1)

        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Robot linear scale correction
        self.linear_scale_correction = 0.1     # Was 0.5
        self.angular_scale_correction = 1

        # TF2 broadcaster
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
            
        # Timer for publishing updates
        self.update_timer = self.create_timer(0.02, self.update_odometry)  # 50Hz
            
        self.get_logger().info('4-Wheel Motor Controller Node initialized with MCU address: 0x16')

    def write_array(self, reg, data):
        try:
            self._device.write_i2c_block_data(self._addr, reg, data)
        except Exception as e:
            self.get_logger().error(f'write_array I2C error: {str(e)}')

    def ctrl_car(self, l_dir, l_speed, r_dir, r_speed):
        try:
            reg = 0x01
            data = [l_dir, l_speed, r_dir, r_speed]
            self.write_array(reg, data)
        except Exception as e:
            self.get_logger().error(f'ctrl_car I2C error: {str(e)}')

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Received cmd_vel - linear.x: {msg.linear.x}, angular: {msg.angular.z}')

        try:
            # Store current velocities for odometry
            self.current_linear_x = msg.linear.x
            self.current_angular_z = msg.angular.z

            # Separate scaling factors
            LINEAR_SCALE = 78  # For forward/backward (This is somewhat tuned to 0.5 m/s)
            ANGULAR_SCALE = 60  # Reduced scale for turning

            # Scale factor for turns (reduces aggressive turning)
            turn_scale = max(0.3, 0.8 - abs(msg.linear.x) / 0.5)  # Adjust dynamically

            # Convert to left and right wheel speeds with separate scaling
            left_speed = msg.linear.x * LINEAR_SCALE - msg.angular.z * ANGULAR_SCALE * turn_scale
            right_speed = msg.linear.x * LINEAR_SCALE + msg.angular.z * ANGULAR_SCALE * turn_scale

            # Determine directions and ensure speed limits
            left_dir = 1 if left_speed >= 0 else 0
            right_dir = 1 if right_speed >= 0 else 0

            left_speed_value = min(abs(int(left_speed)), 120)       # Was 120
            right_speed_value = min(abs(int(right_speed)), 120)     # Was 120

            # Send commands
            self.ctrl_car(left_dir, left_speed_value, right_dir, right_speed_value)

            self.get_logger().debug(f'Sent motor commands - Left: {left_dir},{left_speed_value}, Right: {right_dir},{right_speed_value}')

        except Exception as e:
            self.get_logger().error(f'Failed in cmd_vel processing: {str(e)}')

    def update_odometry(self):
        try:
            current_time = self.get_clock().now()
            dt = (current_time - self.last_update_time).nanoseconds / 1e9
            
            # Apply the scale correction
            actual_linear = self.current_linear_x * self.linear_scale_correction
            actual_angular = self.current_angular_z * self.angular_scale_correction

            # Calculate wheel velocities (all wheels on each side move at the same speed)
            left_velocity = (actual_linear - actual_angular * self.wheel_separation / 2.0) / self.wheel_radius
            right_velocity = (actual_linear + actual_angular * self.wheel_separation / 2.0) / self.wheel_radius
            
            # Linear scalers (Displacement corrector (forward and backwards))
            lin_scale = 0.25

            # Update all wheel positions
            # Front left and rear left
            self.wheel_positions[0] += left_velocity * dt * lin_scale
            self.wheel_positions[2] += left_velocity * dt * lin_scale
            # Front right and rear right
            self.wheel_positions[1] += right_velocity * dt * lin_scale
            self.wheel_positions[3] += right_velocity * dt * lin_scale
            
            # Update robot pose
            delta_x = actual_linear * cos(self.theta) * dt 
            delta_y = actual_linear * sin(self.theta) * dt
            delta_theta = actual_angular * dt
            
            self.x += delta_x * lin_scale
            self.y += delta_y * lin_scale
            self.theta += delta_theta
            
            # Publish joint states for all four wheels
            joint_state = JointState()
            joint_state.header.stamp = current_time.to_msg()
            joint_state.name = self.wheel_names
            joint_state.position = self.wheel_positions
            joint_state.velocity = [left_velocity, right_velocity, left_velocity, right_velocity]
            self.joint_pub.publish(joint_state)
            
            # Publish odometry
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'  # Was base_link
            
            odom.pose.pose.position.x = self.x * lin_scale
            odom.pose.pose.position.y = self.y * lin_scale
            odom.pose.pose.orientation.z = sin(self.theta / 2.0)    
            odom.pose.pose.orientation.w = cos(self.theta / 2.0)
            
            odom.twist.twist.linear.x = self.current_linear_x   * lin_scale
            odom.twist.twist.angular.z = self.current_angular_z
            
            self.odom_pub.publish(odom)
            
            # Broadcast transform
            # t = TransformStamped()
            # t.header.stamp = current_time.to_msg()
            # t.header.frame_id = 'odom'
            # t.child_frame_id = 'base_link'
            # t.transform.translation.x = self.x
            # t.transform.translation.y = self.y
            # t.transform.rotation.z = sin(self.theta / 2.0)
            # t.transform.rotation.w = cos(self.theta / 2.0)
            
            # self.tf_broadcaster.sendTransform(t)
            
            self.last_update_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Failed in odometry update: {str(e)}')

    def stop_motors(self):
        try:
            self._device.write_byte_data(self._addr, 0x02, 0x00)
        except Exception as e:
            self.get_logger().error(f'Failed to stop motors: {str(e)}')

    def test_motors(self):
        self.get_logger().info('Starting motor test')
        try:
            self.ctrl_car(1, 50, 1, 50)
            self.get_logger().info('Sent forward command')
            time.sleep(2)
            self.stop_motors()
            self.get_logger().info('Sent stop command')
        except Exception as e:
            self.get_logger().error(f'Motor test failed: {str(e)}')

def main():
    rclpy.init()
    motor_controller = MotorController()
    # motor_controller.test_motors()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.stop_motors()
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()