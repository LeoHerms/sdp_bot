#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped
import math
import sys
import threading
import time

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        
        # Create action clients for different Nav2 actions
        self.navigate_to_pose_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'
        )
        
        self.follow_waypoints_client = ActionClient(
            self, 
            FollowWaypoints, 
            'follow_waypoints'
        )
        
        # Flag to track if robot is currently navigating
        self.is_navigating = False
        
        self.get_logger().info('Nav2 Action Client initialized')


    def create_pose_stamped(self, x, y, z=0.0, yaw=0.0, frame_id="map"):
        """
        Create a PoseStamped message with the given coordinates and orientation
        """
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        
        # Convert yaw to quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose

    def navigate_to_pose(self, x, y, yaw=0.0, behavior_tree=""):
        """
        Send a goal to the NavigateToPose action server
        """
        # Check if already navigating
        if self.is_navigating:
            self.get_logger().warn('Already navigating to a goal. Ignoring new request.')
            return False
            
        # Wait for server to be available
        self.get_logger().info('Waiting for NavigateToPose action server...')
        if not self.navigate_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available')
            return False
            
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(x, y, yaw=yaw)
        
        # Set behavior tree if provided
        if behavior_tree:
            goal_msg.behavior_tree = behavior_tree
            
        self.get_logger().info(f'Sending navigation goal: x={x}, y={y}, yaw={yaw}')
        
        # Set navigating flag
        self.is_navigating = True
        
        # Send goal and add callbacks
        send_goal_future = self.navigate_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_feedback_callback
        )
        
        send_goal_future.add_done_callback(self.navigate_goal_response_callback)
        return True
        
    def navigate_goal_response_callback(self, future):
        """Handle response from NavigateToPose action server"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            self.is_navigating = False
            return

        self.get_logger().info('Navigation goal accepted')
        
        # Request the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigate_result_callback)
        
    def navigate_result_callback(self, future):
        """Handle result from NavigateToPose action server"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation goal succeeded')
        else:
            self.get_logger().info(f'Navigation goal failed with status: {status}')
            
        # Reset navigating flag
        self.is_navigating = False
        self.get_logger().info('Ready for new goal')
            
    def navigate_feedback_callback(self, feedback_msg):
        """Handle feedback from NavigateToPose action server"""
        feedback = feedback_msg.feedback
        # Extract current pose and remaining distance
        current_pose = feedback.current_pose
        distance_remaining = feedback.distance_remaining
        
        # Log some useful information from the feedback (less frequent to avoid log spam)
        if distance_remaining % 1.0 < 0.1:  # Log roughly every meter
            self.get_logger().info(
                f'Current position: ({current_pose.pose.position.x:.2f}, '
                f'{current_pose.pose.position.y:.2f}), '
                f'Distance remaining: {distance_remaining:.2f}m'
            )
    
    def follow_waypoints(self, waypoints):
        """
        Send a goal to the FollowWaypoints action server
        
        Args:
            waypoints: List of (x, y, yaw) tuples
        """
        # Wait for server to be available
        self.get_logger().info('Waiting for FollowWaypoints action server...')
        if not self.follow_waypoints_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FollowWaypoints action server not available')
            return False
            
        # Create goal
        goal_msg = FollowWaypoints.Goal()
        
        # Convert waypoints to PoseStamped messages
        pose_list = []
        for wp in waypoints:
            if len(wp) == 2:  # (x, y) format
                x, y = wp
                yaw = 0.0
            else:  # (x, y, yaw) format
                x, y, yaw = wp
                
            pose = self.create_pose_stamped(x, y, yaw=yaw)
            pose_list.append(pose)
            
        goal_msg.poses = pose_list
        
        self.get_logger().info(f'Sending {len(waypoints)} waypoints')
        
        # Send goal and add callbacks
        send_goal_future = self.follow_waypoints_client.send_goal_async(
            goal_msg,
            feedback_callback=self.waypoints_feedback_callback
        )
        
        send_goal_future.add_done_callback(self.waypoints_goal_response_callback)
        return True
        
    def waypoints_goal_response_callback(self, future):
        """Handle response from FollowWaypoints action server"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Waypoints goal rejected')
            return

        self.get_logger().info('Waypoints goal accepted')
        
        # Request the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.waypoints_result_callback)
        
    def waypoints_result_callback(self, future):
        """Handle result from FollowWaypoints action server"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Waypoints navigation succeeded')
            # Check if there were any missed waypoints
            if result.missed_waypoints:
                self.get_logger().warning(
                    f'Missed waypoints: {result.missed_waypoints}'
                )
        else:
            self.get_logger().info(f'Waypoints navigation failed with status: {status}')
            
    def waypoints_feedback_callback(self, feedback_msg):
        """Handle feedback from FollowWaypoints action server"""
        feedback = feedback_msg.feedback
        # Get current waypoint being executed
        current_waypoint = feedback.current_waypoint
        
        self.get_logger().info(f'Currently executing waypoint: {current_waypoint}')

def main(args=None):
    rclpy.init(args=args)
    client = Nav2ActionClient()
    
    # Create a separate thread for spinning the node
    spin_thread = threading.Thread(target=lambda: rclpy.spin(client))
    spin_thread.daemon = True
    spin_thread.start()
    
    try:
        # Interactive mode
        if len(sys.argv) > 1 and sys.argv[1] == 'interactive':
            print("Interactive Navigation Client")
            print("Enter coordinates as 'x y [yaw]' or 'waypoints' or 'quit'")
            
            while True:
                # Wait until robot is not navigating before asking for new goal
                while client.is_navigating:
                    time.sleep(0.5)
                    
                command = input("\nEnter command ('x y [yaw]', 'waypoints', or 'quit'): ")
                
                if command.lower() == 'quit':
                    break
                    
                elif command.lower() == 'waypoints':
                    # Example waypoints [(x, y, yaw), ...]
                    waypoints = [
                        (1.0, 1.0, 0.0),
                        (2.0, 2.0, 1.57),
                        (1.0, 3.0, 3.14)
                    ]
                    client.follow_waypoints(waypoints)
                    
                else:
                    # Parse coordinates
                    try:
                        parts = command.split()
                        if len(parts) >= 2:
                            x = float(parts[0])
                            y = float(parts[1])
                            yaw = float(parts[2]) if len(parts) > 2 else 0.0
                            client.navigate_to_pose(x, y, yaw)
                        else:
                            print("Invalid format. Use 'x y [yaw]'")
                    except ValueError:
                        print("Invalid coordinates. Use numbers for x, y, and yaw")
        
        # Command line mode (for backward compatibility)
        elif len(sys.argv) > 2:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
            
            client.navigate_to_pose(x, y, yaw)
            # In command line mode, wait for goal to complete
            while client.is_navigating:
                time.sleep(0.5)
                
        # Waypoints mode
        elif len(sys.argv) == 2 and sys.argv[1] == 'waypoints':
            waypoints = [
                (1.0, 1.0, 0.0),
                (2.0, 2.0, 1.57),
                (1.0, 3.0, 3.14)
            ]
            client.follow_waypoints(waypoints)
            # Wait for waypoints to complete
            while client.is_navigating:
                time.sleep(0.5)
                
        else:
            print('Usage:')
            print('  python3 nav2_client_simple.py interactive')
            print('  python3 nav2_client_simple.py x y [yaw]')
            print('  python3 nav2_client_simple.py waypoints')
            
    except KeyboardInterrupt:
        print("Navigation client stopped by keyboard interrupt")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()