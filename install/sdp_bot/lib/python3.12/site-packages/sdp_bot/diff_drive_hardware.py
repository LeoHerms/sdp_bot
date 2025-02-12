from hardware_interface import SystemInterface
import time
import smbus
from math import pi

class DiffDriveHardwareInterface(SystemInterface):
    def __init__(self, context):
        super().__init__()
        
        # Hardware/robot parameters
        self.wheel_radius = 0.03175  # meters
        self.wheel_separation = 0.127  # meters
        
        # Initialize interface states
        self._position_states = [0.0, 0.0]  # left, right wheel positions
        self._velocity_states = [0.0, 0.0]  # left, right wheel velocities
        self._velocity_commands = [0.0, 0.0]  # left, right wheel commands
        
        # Initialize I2C
        self._addr = 0x16
        self._device = smbus.SMBus(1)
        
        # For position estimation
        self._last_time = time.time()

    def on_configure(self, state):
        # Declare joints
        self.joint_names = ['left_joint', 'right_joint']
        
        # Set up interface types
        self._position_interfaces = [
            self.joint_names[0] + '/position',
            self.joint_names[1] + '/position'
        ]
        self._velocity_interfaces = [
            self.joint_names[0] + '/velocity',
            self.joint_names[1] + '/velocity'
        ]
        self._command_interfaces = [
            self.joint_names[0] + '/velocity',
            self.joint_names[1] + '/velocity'
        ]
        
        return True

    def on_activate(self, state):
        # Reset states and commands
        self._position_states = [0.0, 0.0]
        self._velocity_states = [0.0, 0.0]
        self._velocity_commands = [0.0, 0.0]
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
        
        # Update position estimation based on last commanded velocity
        self._position_states[0] += self._velocity_states[0] * dt
        self._position_states[1] += self._velocity_states[1] * dt
        
        self._last_time = current_time
        return True

    def write(self, time, period):
        try:
            # Update current velocity states from commands
            self._velocity_states = self._velocity_commands.copy()
            
            # Convert commanded velocities to motor commands
            left_speed = int(abs(self._velocity_commands[0]) * 120)  # Scale to 0-120
            right_speed = int(abs(self._velocity_commands[1]) * 120)
            
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
        if name == self._position_interfaces[0]:
            return self._position_states[0]
        elif name == self._position_interfaces[1]:
            return self._position_states[1]
        else:
            raise RuntimeError(f'Unknown position interface: {name}')

    def get_velocity_state_interface_value(self, name):
        if name == self._velocity_interfaces[0]:
            return self._velocity_states[0]
        elif name == self._velocity_interfaces[1]:
            return self._velocity_states[1]
        else:
            raise RuntimeError(f'Unknown velocity interface: {name}')

    def set_command_interface_value(self, name, value):
        if name == self._command_interfaces[0]:
            self._velocity_commands[0] = value
        elif name == self._command_interfaces[1]:
            self._velocity_commands[1] = value
        else:
            raise RuntimeError(f'Unknown command interface: {name}')