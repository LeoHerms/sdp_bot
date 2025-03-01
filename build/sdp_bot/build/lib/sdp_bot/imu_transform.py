#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster

# This node listens to IMU data and broadcasts a transform from the IMU frame to the base_link frame
# Allows the robot to move with the IMU orientation
# Question: Should we publish this for the nav2 stack or for the robot_localization node?

class ImuTfBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        
        # Create a TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to IMU data
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',  # Your IMU topic
            self.imu_callback,
            10)
            
    def imu_callback(self, msg):
        # Create transform
        t = TransformStamped()
        
        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'
        
        # Set translation (usually 0 for IMU orientation only)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.07747  # Height from URDF
        
        # Set rotation from IMU
        t.transform.rotation = msg.orientation
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuTfBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()