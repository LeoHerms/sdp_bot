#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import serial
from math import sqrt

from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu, MagneticField

class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        
        # Get parameter values
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        # Configure serial connection
        try:
            self.serial = serial.Serial(port, baud)
            self.get_logger().info(f'Connected to {port} at {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {str(e)}')
            raise
        
        # Configure QoS profile
        qos = QoSProfile(
            depth=10,
            # Using default reliability (RELIABLE) for compatibility with standard subscribers
        )
        
        # Create publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', qos)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', qos)
        
        # Create timer for reading serial data (1000Hz)
        self.create_timer(0.001, self.read_serial_data)
        
        # Initialize data storage
        self.quaternion = None
        self.orientation = None
        self.gyro = None
        self.mag = None
        self.accel = None
        
        self.get_logger().info('BNO055 node initialized')
    
    def read_serial_data(self):
        """Read and process data from serial port"""
        if self.serial.in_waiting:
            try:
                line = self.serial.readline().decode('utf-8').strip()
                if line.startswith('D:'):
                    self.process_sensor_data(line)
                elif line.startswith('E:'):
                    self.handle_error(line)
            except Exception as e:
                self.get_logger().error(f'Error reading serial data: {str(e)}')
    
    def process_sensor_data(self, line):
        """Process incoming sensor data line"""
        parts = line.split(':')
        if len(parts) < 3:
            return
            
        data_type = parts[1]
        values = [float(x) for x in parts[2:]]
        
        # Store data based on type
        if data_type == 'Q':  # Quaternion
            self.quaternion = values
            self.publish_imu_data()
        elif data_type == 'O':  # Orientation (Euler)
            self.orientation = values
        elif data_type == 'G':  # Gyroscope
            self.gyro = values
        elif data_type == 'M':  # Magnetometer
            self.mag = values
            self.publish_mag_data()
        elif data_type == 'A':  # Accelerometer
            self.accel = values
    
    def publish_imu_data(self):
        """Publish IMU data if all required components are available"""
        if all(x is not None for x in [self.quaternion, self.gyro, self.accel]):
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.get_parameter('frame_id').value
            
            # Set orientation (quaternion)
            msg.orientation.w = self.quaternion[0]
            msg.orientation.x = self.quaternion[1]     
            msg.orientation.y = self.quaternion[2]      
            msg.orientation.z = self.quaternion[3]    
            
            # Set angular velocity (gyro)
            msg.angular_velocity.x = self.gyro[0]       
            msg.angular_velocity.y = self.gyro[1]      
            msg.angular_velocity.z = self.gyro[2]       
            
            # Set linear acceleration
            msg.linear_acceleration.x = self.accel[0]   
            msg.linear_acceleration.y = self.accel[1]   
            msg.linear_acceleration.z = self.accel[2] 
            
            # Set covariance matrices (optional)
            # msg.orientation_covariance = [-1.0] * 9  # -1 indicates unknown
            # msg.angular_velocity_covariance = [-1.0] * 9
            # Small covariance for orientation (high confidence)
            msg.orientation_covariance = [0.0001, 0, 0,
                                        0, 0.0001, 0,
                                        0, 0, 0.0001]

            # Higher covariance for angular velocity (lower confidence)
            msg.angular_velocity_covariance = [0.001, 0, 0,
                                            0, 0.001, 0,
                                            0, 0, 0.001]
            msg.linear_acceleration_covariance = [-1.0] * 9
            
            self.imu_pub.publish(msg)
    
    def publish_mag_data(self):
        """Publish magnetometer data"""
        if self.mag is not None:
            msg = MagneticField()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.get_parameter('frame_id').value
            
            msg.magnetic_field.x = self.mag[0]
            msg.magnetic_field.y = self.mag[1]
            msg.magnetic_field.z = self.mag[2]
            
            # Set covariance matrix (optional)
            msg.magnetic_field_covariance = [-1.0] * 9  # -1 indicates unknown
            
            self.mag_pub.publish(msg)
    
    def handle_error(self, error_line):
        """Handle error messages from the Arduino"""
        error_code = error_line.split(':')[1]
        if error_code == 'NO_SENSOR':
            self.get_logger().error('BNO055 sensor not detected')
        else:
            self.get_logger().error(f'Unknown error: {error_code}')

def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        # Clean up
        if hasattr(node, 'serial'):
            node.serial.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()