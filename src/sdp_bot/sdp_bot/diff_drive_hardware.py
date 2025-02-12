from hardware_interface import SystemInterface
import rclpy
from rclpy.lifecycle import Node
import time
import smbus
from math import pi

class DiffDriveHardwareInterface(SystemInterface):
    def __init__(self, context):
        super().__init__()
        
        # Hardware/robot parameters
        self.wheel_radius = 0.03175  # meters
        self.wheel_separation = 0.127  # meters
        
        # Initialize interface states - now for 4 wheels
        self._position_states = [0.0, 0.0, 0.0, 0.0]  # left, right, front_left, front_right positions
        self._velocity_states = [0.0, 0.0, 0.0, 0.0]  # left, right, front_left, front_right velocities
        self._velocity_commands = [0.0, 0.0, 0.0, 0.0]  # left, right, front_left, front_right commands
        
        # Initialize I2C
        self._addr = 0x16
        self._device = smbus.SMBus(1)
        
        # For position estimation
        self._last_time = time.time()

    def on_configure(self, state):
        # Declare joints
        self.joint_names = ['base_back_left_wheel_joint', 'base_back_right_wheel_joint', 'base_front_left_wheel_joint', 'base_front_right_wheel_joint']
        
        # Set up interface types
        self._position_interfaces = [
            self.joint_names[0] + '/position',
            self.joint_names[1] + '/position',
            self.joint_names[2] + '/position',
            self.joint_names[3] + '/position'
        ]
        self._velocity_interfaces = [
            self.joint_names[0] + '/velocity',
            self.joint_names[1] + '/velocity',
            self.joint_names[2] + '/velocity',
            self.joint_names[3] + '/velocity'
        ]
        self._command_interfaces = [
            self.joint_names[0] + '/velocity',
            self.joint_names[1] + '/velocity',
            self.joint_names[2] + '/velocity',
            self.joint_names[3] + '/velocity'
        ]
        
        return True

    def on_activate(self, state):
        # Reset states and commands
        self._position_states = [0.0, 0.0, 0.0, 0.0]
        self._velocity_states = [0.0, 0.0, 0.0, 0.0]
        self._velocity_commands = [0.0, 0.0, 0.0, 0.0]
        self._last_time = time.time()
        return True

    def on_deactivate(self, state):
        # Stop motors
        try:
            self._device.write_byte_data(self._addr, 0x02, 0x00)
        except Exception as e:
            print(f'Failed to stop motors: {str(e)}')
        return True

    def read(self, time, period):
        # Calculate position based on commanded velocities (open-loop)
        current_time = time.time()
        dt = current_time - self._last_time
        
        # Update position estimation for all wheels
        for i in range(4):
            self._position_states[i] += self._velocity_states[i] * dt
        
        self._last_time = current_time
        return True

    def write(self, time, period):
        try:
            # Update current velocity states from commands
            self._velocity_states = self._velocity_commands.copy()
            
            # For skid steering, we use the same command for both wheels on each side
            left_speed = int(abs(self._velocity_commands[0]) * 120)  # Scale to 0-120
            right_speed = int(abs(self._velocity_commands[1]) * 120)
            
            # Copy commands to front wheels
            self._velocity_states[2] = self._velocity_states[0]  # Front left = left
            self._velocity_states[3] = self._velocity_states[1]  # Front right = right
            
            # Determine directions
            left_dir = 1 if self._velocity_commands[0] >= 0 else 0
            right_dir = 1 if self._velocity_commands[1] >= 0 else 0
            
            # Send commands via I2C
            reg = 0x01
            data = [left_dir, min(left_speed, 120), right_dir, min(right_speed, 120)]
            self._device.write_i2c_block_data(self._addr, reg, data)
            
            return True
        except Exception as e:
            print(f'Failed to write commands: {str(e)}')
            return False

    def get_name_prefix(self):
        return 'DiffDriveHardwareInterface'

    def get_state_interface_names(self):
        return self._position_interfaces + self._velocity_interfaces

    def get_command_interface_names(self):
        return self._command_interfaces

    def get_position_state_interface_value(self, name):
        for i, interface in enumerate(self._position_interfaces):
            if name == interface:
                return self._position_states[i]
        raise RuntimeError(f'Unknown position interface: {name}')

    def get_velocity_state_interface_value(self, name):
        for i, interface in enumerate(self._velocity_interfaces):
            if name == interface:
                return self._velocity_states[i]
        raise RuntimeError(f'Unknown velocity interface: {name}')

    def set_command_interface_value(self, name, value):
        for i, interface in enumerate(self._command_interfaces):
            if name == interface:
                self._velocity_commands[i] = value
                return
        raise RuntimeError(f'Unknown command interface: {name}')