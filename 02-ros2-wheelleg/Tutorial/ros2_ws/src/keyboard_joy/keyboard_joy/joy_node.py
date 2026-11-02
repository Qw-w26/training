#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from pynput import keyboard
import threading
import yaml
import os
import select
import sys
import termios
import time
import tty
from ament_index_python.packages import get_package_share_directory

class KeyboardJoy(Node):
    def __init__(self):
        super().__init__('keyboard_joy')

        # Declare parameter for configuration file path
        self.declare_parameter('config', '')

        # Load key mappings and other parameters from the YAML file
        self.load_key_mappings()

        # Print a message to indicate that the node has started
        self.get_logger().info("KeyboardJoy Node Started")
        self.get_logger().info(f"Loaded axis mappings: {self.axis_mappings}")
        self.get_logger().info(f"Loaded button mappings: {self.button_mappings}")
        self.get_logger().info(f"Axis increment rate: {self.axis_increment_rate}, Axis increment step: {self.axis_increment_step}")

        # Create a publisher for the Joy message
        self.joy_publisher = self.create_publisher(Joy, 'joy', 10)

        # Initialize Joy message with the correct number of axes and buttons
        max_axis_index = max(self.axis_mappings.values(), key=lambda x: x[0])[0] if self.axis_mappings else 0
        max_button_index = max(self.button_mappings.values()) if self.button_mappings else 0

        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * (max_axis_index + 1)
        self.joy_msg.buttons = [0] * (max_button_index + 1)

        # Track active axes for gradual updates
        self.active_axes = {}
        self.sticky_axes = {}

        # Create a lock for thread-safe updates
        self.lock = threading.Lock()

        self.listener = None
        self.terminal_fd = None
        self.terminal_settings = None
        self.terminal_deadlines = {}
        self.terminal_key_timeout = 0.6
        self.listener_stop = threading.Event()
        if sys.stdin.isatty():
            self.terminal_fd = sys.stdin.fileno()
            self.terminal_settings = termios.tcgetattr(self.terminal_fd)
            tty.setcbreak(self.terminal_fd)
            self.get_logger().info(
                'Reading keys here: WASD/arrows move, 4/5 adjust leg length')
            self.listener_thread = threading.Thread(
                target=self.read_terminal_keys, daemon=True)
        else:
            self.listener = keyboard.Listener(
                on_press=self.on_press, on_release=self.on_release)
            self.listener_thread = threading.Thread(
                target=self.listener.run, daemon=True)
        self.listener_thread.start()

        # Create a timer to publish Joy messages at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_joy)

        # Create a timer for gradually updating active axes
        self.increment_timer = self.create_timer(self.axis_increment_rate, self.update_active_axes)

    def load_key_mappings(self):
        """Load key mappings and parameters from a YAML file."""
        config_file_path = self.get_parameter('config').get_parameter_value().string_value

        if not config_file_path:
            config_file_path = os.path.join(get_package_share_directory('keyboard_joy'), 'config', 'key_mappings.yaml')

        try:
            with open(config_file_path, 'r') as file:
                key_mappings = yaml.safe_load(file)
        except FileNotFoundError:
            self.get_logger().error(f"Configuration file not found: {config_file_path}")
            key_mappings = {}

        self.axis_mappings = key_mappings.get('axes', {})
        self.button_mappings = key_mappings.get('buttons', {})

        # Load axis_increment_rate, axis_increment_step from the 'parameters' section
        parameters = key_mappings.get('parameters', {})

        self.axis_increment_rate = parameters.get('axis_increment_rate', 0.1)  # Default to 0.1 if not specified
        self.axis_increment_step = parameters.get('axis_increment_step', 0.05)  # Default to 0.05 if not specified

    def on_press(self, key):
        """Callback for keyboard key press events."""
        self.apply_key_press(self.key_to_string(key))

    def apply_key_press(self, key_str):
        with self.lock:
            if key_str in self.axis_mappings:
                axis, value, mode = self.axis_mappings[key_str]
                if mode == 'sticky':
                    # In sticky mode, increment the axis value gradually
                    self.sticky_axes[axis] = self.sticky_axes.get(axis, 0.0) + value * self.axis_increment_step
                    # Clamp the value and round to 4 decimal places
                    self.joy_msg.axes[axis] = round(max(min(self.sticky_axes[axis], 1.0), -1.0), 4)
                else:
                    if self.terminal_fd is not None:
                        self.joy_msg.axes[axis] = value
                        self.terminal_deadlines[('axis', axis)] = (
                            time.monotonic() + self.terminal_key_timeout)
                    else:
                        # Gradually change axes while a desktop key is held.
                        self.active_axes[axis] = value
            elif key_str in self.button_mappings:
                button_index = self.button_mappings[key_str]
                self.joy_msg.buttons[button_index] = 1
                if self.terminal_fd is not None:
                    self.terminal_deadlines[('button', button_index)] = (
                        time.monotonic() + self.terminal_key_timeout)

    def on_release(self, key):
        """Callback for keyboard key release events."""
        with self.lock:
            key_str = self.key_to_string(key)
            if key_str in self.axis_mappings:
                axis, _, mode = self.axis_mappings[key_str]
                if axis in self.active_axes:
                    del self.active_axes[axis]
                if mode != 'sticky':
                    self.joy_msg.axes[axis] = 0.0  # Reset to zero on release for non-sticky mode
            elif key_str in self.button_mappings:
                button_index = self.button_mappings[key_str]
                self.joy_msg.buttons[button_index] = 0

    def key_to_string(self, key):
        """Convert keyboard key to string for parameter lookup."""
        if hasattr(key, 'char') and key.char is not None:
            return key.char
        elif hasattr(key, 'name') and key.name is not None:
            return f'Key.{key.name}'
        else:
            return str(key)

    def publish_joy(self):
        """Publish the Joy message based on current keyboard state."""
        with self.lock:
            self.joy_msg.header.stamp = self.get_clock().now().to_msg()
            self.joy_publisher.publish(self.joy_msg)

    def update_active_axes(self):
        """Update the active axes gradually based on the increment step."""
        with self.lock:
            now = time.monotonic()
            for (kind, index), deadline in list(self.terminal_deadlines.items()):
                if now >= deadline:
                    if kind == 'axis':
                        self.active_axes.pop(index, None)
                        self.joy_msg.axes[index] = 0.0
                    else:
                        self.joy_msg.buttons[index] = 0
                    del self.terminal_deadlines[(kind, index)]

            for axis, target_value in self.active_axes.items():
                current_value = self.joy_msg.axes[axis]
                # Increment towards the target (1.0 or -1.0) and round to 4 decimal places
                if target_value > 0:
                    self.joy_msg.axes[axis] = round(min(current_value + self.axis_increment_step, target_value), 4)
                else:
                    self.joy_msg.axes[axis] = round(max(current_value - self.axis_increment_step, target_value), 4)

    def read_terminal_keys(self):
        """Read terminal key presses, expiring them because terminals lack key-up events."""
        fd = self.terminal_fd
        while not self.listener_stop.is_set():
            ready, _, _ = select.select([fd], [], [], 0.05)
            if not ready:
                continue

            char = os.read(fd, 1).decode(errors='ignore')
            if char == '\x1b':
                sequence = ''
                deadline = time.monotonic() + 0.05
                while time.monotonic() < deadline:
                    ready, _, _ = select.select([fd], [], [], 0.01)
                    if not ready:
                        continue
                    sequence += os.read(fd, 1).decode(errors='ignore')
                    if sequence and sequence[-1] in 'ABCD':
                        break
                key_str = {'A': 'Key.up', 'B': 'Key.down',
                           'C': 'Key.right', 'D': 'Key.left'}.get(sequence[-1:])
            else:
                key_str = char.lower()

            if key_str:
                self.apply_key_press(key_str)

    def destroy_node(self):
        """Ensure the listener thread is properly stopped."""
        self.listener_stop.set()
        if self.listener is not None:
            self.listener.stop()
        self.listener_thread.join(timeout=1.0)
        if self.terminal_fd is not None and self.terminal_settings is not None:
            termios.tcsetattr(self.terminal_fd, termios.TCSADRAIN,
                              self.terminal_settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    keyboard_joy = KeyboardJoy()

    try:
        rclpy.spin(keyboard_joy)
    except KeyboardInterrupt:
        pass

    keyboard_joy.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
