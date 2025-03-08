#!/usr/bin/env python3

import rclpy
import threading
import json
import time
import signal
import sys
from flask import Flask, request, jsonify, Response
from flask_cors import CORS

from nav2_client_simple import Nav2ActionClient

class Nav2FlaskServer:
    """
    Flask server that interfaces with Nav2 action client
    """
    def __init__(self, nav2_client, host='0.0.0.0', port=8080, debug=False):
        self.nav2_client = nav2_client
        self.host = host
        self.port = port
        self.debug = debug
        
        # Create Flask app
        self.app = Flask(__name__)
        # Enable CORS for all routes
        CORS(self.app)
        
        # Register routes
        self.register_routes()
        
        # Server thread
        self.server_thread = None
        self.running = False
    
    def register_routes(self):
        """Register all API routes"""

        @self.app.route('/status', methods=['GET'])
        def get_status():
            """Get the current status of the robot"""
            try:
                status = {
                    'status': 'ok',
                    'is_navigating': self.nav2_client.is_navigating
                }
                return jsonify(status)
            except Exception as e:
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/navigate', methods=['POST'])
        def navigate():
            """Send a navigation goal to the robot"""
            try:
                data = request.json
                
                # Check required fields
                required_fields = ['x', 'y']
                for field in required_fields:
                    if field not in data:
                        return jsonify({'error': f'Missing required field: {field}'}), 400
                
                x = float(data['x'])
                y = float(data['y'])
                yaw = float(data.get('yaw', 0.0))
                behavior_tree = data.get('behavior_tree', '')
                
                # Check if the robot is already navigating
                if self.nav2_client.is_navigating:
                    return jsonify({
                        'status': 'error',
                        'message': 'Robot is already navigating to a goal'
                    }), 409  # 409 Conflict
                
                # Send the navigation goal
                success = self.nav2_client.navigate_to_pose(x, y, yaw, behavior_tree)
                
                if success:
                    return jsonify({
                        'status': 'success',
                        'message': f'Navigation goal sent: x={x}, y={y}, yaw={yaw}'
                    })
                else:
                    return jsonify({
                        'status': 'error',
                        'message': 'Failed to send navigation goal'
                    }), 500
                    
            except ValueError as ve:
                return jsonify({'error': f'Invalid coordinate value: {str(ve)}'}), 400
            except Exception as e:
                return jsonify({'error': f'Unexpected error: {str(e)}'}), 500
        

        @self.app.route('/', methods=['GET'])
        def index():
            """Serve a simple status page"""
            html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>Nav2 API Server</title>
                <style>
                    body { font-family: Arial, sans-serif; max-width: 800px; margin: 0 auto; padding: 20px; }
                    h1 { color: #333; }
                    .endpoint { background: #f5f5f5; padding: 15px; margin-bottom: 15px; border-radius: 5px; }
                    .method { display: inline-block; padding: 5px 10px; border-radius: 3px; color: white; }
                    .get { background: #4CAF50; }
                    .post { background: #2196F3; }
                </style>
            </head>
            <body>
                <h1>Nav2 API Server</h1>
                <p>The server is running properly. Available endpoints:</p>
                
                <div class="endpoint">
                    <span class="method get">GET</span>
                    <strong>/status</strong>
                    <p>Get the current status of the robot</p>
                </div>
                
                <div class="endpoint">
                    <span class="method post">POST</span>
                    <strong>/navigate</strong>
                    <p>Send a navigation goal to the robot</p>
                    <p>Example payload: <code>{"x": 1.0, "y": 2.0, "yaw": 0.0}</code></p>
                </div>
            </body>
            </html>
            """
            return Response(html, mimetype='text/html')
    
    def start(self):
        """Start the Flask server in a separate thread"""
        if self.running:
            print("Server is already running")
            return
        
        self.running = True
        
        print(f"HTTP server starting on {self.host}:{self.port}")
        
        # Start the server in a separate thread
        self.server_thread = threading.Thread(
            target=self.app.run,
            kwargs={
                'host': self.host,
                'port': self.port,
                'debug': self.debug,
                'use_reloader': False  # Disable reloader to avoid creating multiple threads
            }
        )
        self.server_thread.daemon = True
        self.server_thread.start()
        
        print(f"HTTP server started on {self.host}:{self.port}")
    
    def stop(self):
        """Stop the Flask server"""
        if not self.running:
            return
            
        self.running = False
        print("Stopping server...")
        # The server will be terminated when the main process exits


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create the Nav2 action client
    nav2_client = Nav2ActionClient()
    
    # Create a separate thread for spinning the ROS2 node
    spin_thread = threading.Thread(target=lambda: rclpy.spin(nav2_client))
    spin_thread.daemon = True
    spin_thread.start()
    
    # Create and start the Flask server (no SSL)
    server = Nav2FlaskServer(
        nav2_client,
        host='0.0.0.0',
        port=8080,  # Standard HTTP port
        debug=False  # Set to True for development
    )
    
    try:
        def signal_handler(sig, frame):
            print("Shutting down...")
            server.stop()
            nav2_client.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
            
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Start the server
        server.start()
        
        # Keep the main thread alive
        while True:
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        server.stop()
        nav2_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()