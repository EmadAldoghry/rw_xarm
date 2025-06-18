#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import evdev
from evdev import InputDevice, categorize, ecodes
import threading
import yaml
import os
from pathlib import Path

class GamepadController(Node):
    def __init__(self):
        super().__init__('gamepad_controller')

        # Create publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize control state
        self.is_active = False  # Control state

        # Find gamepad
        self.gamepad = self.find_gamepad()
        if not self.gamepad:
            self.get_logger().error('No gamepad found!')
            return

        # Print gamepad capabilities
        self.print_gamepad_info()

        # Uncomment this line to debug gamepad buttons
        # self.debug_gamepad()

        # Load control configuration
        self.config = self.load_config()

        # Initialize velocities
        self.linear_speed = 0.5  # Default linear speed
        self.angular_speed = 1.0  # Default angular speed

        # Initialize current values
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.speed_multiplier = 1.0

        # Start gamepad reading thread
        self.running = True
        self.gamepad_thread = threading.Thread(target=self.read_gamepad)
        self.gamepad_thread.start()

        # Create timer for publishing commands
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Gamepad controller started')
        self.get_logger().info(f'Using gamepad: {self.gamepad.name}')
        self.print_controls()

    def debug_gamepad(self):
        """Print all gamepad events to help identify button codes."""
        try:
            self.get_logger().info("Starting gamepad debug mode. Press buttons to see their codes...")
            for event in self.gamepad.read_loop():
                if event.type != ecodes.EV_SYN:  # Ignore sync events
                    self.get_logger().info(f"Event: type={event.type}, code={event.code}, value={event.value}")
        except KeyboardInterrupt:
            pass

    def print_gamepad_info(self):
        self.get_logger().info("\nGamepad Capabilities:")
        self.get_logger().info(f"Name: {self.gamepad.name}")
        self.get_logger().info("Available buttons:")
        for code, type_ in self.gamepad.capabilities(verbose=True).items():
            self.get_logger().info(f"{code}: {type_}")

        # Print current configuration
        self.get_logger().info("\nCurrent Configuration:")
        if hasattr(self, 'config'):
            for key, value in self.config['controls'].items():
                self.get_logger().info(f"{key}: {value}")

    def find_gamepad(self):
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            self.get_logger().info(f"Found device: {device.name}")
            if any(keyword in device.name.lower() for keyword in ['joy', 'gamepad', 'controller']):
                return device
        return None

    def load_config(self):
        # Default configuration
        default_config = {
            'controls': {
                'linear_axis': ecodes.ABS_HAT0Y,    # Left stick up/down
                'angular_axis': ecodes.ABS_HAT0X,   # Left stick left/right
                'speed_up': ecodes.BTN_PINKIE,      # Right trigger button
                'speed_down': ecodes.BTN_BASE2,     # Left trigger button
                'activate_button': ecodes.BTN_TOP2,  # Start button to activate/deactivate
                'deadzone': 3000,                   # Deadzone for analog sticks
                'max_speed_linear': 1.0,            # Maximum linear speed
                'max_speed_angular': 2.0            # Maximum angular speed
            }
        }

        # Try to load config file
        config_path = Path.home() / '.ros' / 'gamepad_control_config.yaml'

        if config_path.exists():
            try:
                with open(config_path, 'r') as f:
                    loaded_config = yaml.safe_load(f)
                    # Merge with default config to ensure all keys exist
                    if 'controls' in loaded_config:
                        default_config['controls'].update(loaded_config['controls'])
                    return default_config
            except Exception as e:
                self.get_logger().warning(f"Error loading config file: {e}. Using default configuration.")
                return default_config
        else:
            # Create default config file
            try:
                os.makedirs(config_path.parent, exist_ok=True)
                with open(config_path, 'w') as f:
                    yaml.dump(default_config, f)
            except Exception as e:
                self.get_logger().warning(f"Error creating config file: {e}")
            return default_config

    def print_controls(self):
        self.get_logger().info('\nControl Configuration:')
        self.get_logger().info('Press START button (BTN_TOP2) to activate/deactivate control')
        self.get_logger().info('D-pad or Left stick up/down: Forward/Backward')
        self.get_logger().info('D-pad or Left stick left/right: Turn left/right')
        self.get_logger().info('BTN_PINKIE: Increase speed')
        self.get_logger().info('BTN_BASE2: Decrease speed')
        self.get_logger().info(f"Current linear speed multiplier: {self.linear_speed}")
        self.get_logger().info(f"Current angular speed multiplier: {self.angular_speed}")
        self.get_logger().info(f"Control active: {self.is_active}")

    def map_axis_value(self, value):
        # Map the axis value to a float between -1 and 1
        # For D-pad (HAT) values are typically -1, 0, 1
        # For analog sticks, values are typically -32768 to 32767
        if abs(value) > self.config['controls']['deadzone']:
            if abs(value) <= 1:  # D-pad values
                return float(value)
            else:  # Analog stick values
                return value / 32768.0
        return 0.0

    def read_gamepad(self):
        try:
            for event in self.gamepad.read_loop():
                if not self.running:
                    break

                # Debug output
                self.get_logger().debug(f"Event: type={event.type}, code={event.code}, value={event.value}")

                controls = self.config['controls']

                # Handle activation button
                if event.type == ecodes.EV_KEY and event.code == controls.get('activate_button') and event.value == 1:
                    self.is_active = not self.is_active
                    self.get_logger().info(f"Control {'activated' if self.is_active else 'deactivated'}")
                    continue

                # Only process other controls if activated
                if not self.is_active:
                    continue

                if event.type == ecodes.EV_ABS:
                    if event.code == controls['linear_axis']:
                        self.current_linear_x = -self.map_axis_value(event.value) * controls['max_speed_linear']
                        self.get_logger().debug(f"Linear X: {self.current_linear_x}")

                    elif event.code == controls['angular_axis']:
                        self.current_angular_z = -self.map_axis_value(event.value) * controls['max_speed_angular']
                        self.get_logger().debug(f"Angular Z: {self.current_angular_z}")

                elif event.type == ecodes.EV_KEY:
                    if event.code == controls['speed_up'] and event.value == 1:
                        self.speed_multiplier = min(self.speed_multiplier + 0.1, 2.0)
                        self.get_logger().info(f'Speed multiplier increased to: {self.speed_multiplier:.1f}')

                    elif event.code == controls['speed_down'] and event.value == 1:
                        self.speed_multiplier = max(self.speed_multiplier - 0.1, 0.1)
                        self.get_logger().info(f'Speed multiplier decreased to: {self.speed_multiplier:.1f}')

        except Exception as e:
            self.get_logger().error(f'Gamepad error: {str(e)}')
            self.get_logger().error(f'Event details - type: {event.type}, code: {event.code}, value: {event.value}')

    def timer_callback(self):
        if self.is_active:
            twist = Twist()
            twist.linear.x = self.current_linear_x * self.speed_multiplier
            twist.angular.z = self.current_angular_z * self.speed_multiplier
            self.publisher.publish(twist)
            self.get_logger().debug(f"Publishing - linear: {twist.linear.x}, angular: {twist.angular.z}")
        else:
            # Publish zero velocity when inactive
            self.publisher.publish(Twist())

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'gamepad_thread'):
            self.gamepad_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = GamepadController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()