
# ROS2 Keyboard to Joy

`keyboard_joy` is an optional ROS 2 package that converts keyboard input into
`sensor_msgs/msg/Joy` messages on `/joy`. A physical gamepad can use the
standard `joy_node` directly and does not require this package.

This package is included in `Tutorial/ros2_ws`. It publishes `sensor_msgs/msg/Joy`
on `/joy`; in this workspace the wheel-leg controller reads the standard Xbox
layout: `axes[1]` for forward/reverse motion, `axes[3]` for yaw, and buttons
4/5 for leg-length adjustment. Arrow keys and WASD use the same axis layout;
keys `4`/`5` control leg length.

## Installation

1. **Install ROS2**: Follow the official ROS2 installation guide for your operating system.

2. **Install the `pynput` Library**:

   This package requires `pynput` to capture keyboard input. Install it using pip:

   ```bash
   pip3 install pynput
   ```

3. **Build the Optional Package**:

   Build your workspace to compile the package:

   ```bash
   cd ~/your_ros2_workspace
   colcon build --packages-select keyboard_joy
   ```

4. **Source the Workspace**:

   Source your workspace to overlay your environment:

   ```bash
   source install/setup.bash
   ```

## Usage

To run the `keyboard_joy` node using the default configuration (as stored in `/config/key_mappings.yaml`), execute the following command:

```bash
ros2 run keyboard_joy joy_node
```

When started from an interactive terminal, the node reads keys from that
terminal directly. Keep that terminal focused and press WASD or the arrow keys
to move; press `4`/`5` to adjust leg length. Keys are not echoed, and their
axis/button values return to zero 0.6 seconds after key repeat stops. In a
non-interactive session, the node falls back to `pynput` desktop capture.

However, if you want to use a custom YAML configuration file for your project, run the following command:

```bash
ros2 launch keyboard_joy keyboard_joy.launch.py config:=/your_path/your_config_file.yaml
```

Replace `/your_path/your_config_file.yaml` with the path to your custom YAML file.

## Configuration

The key mappings for the joystick simulation are configured using a YAML file which allows you to customize which keyboard keys map to specific joystick axes and buttons.

### Example YAML Configuration

```yaml
# Config for mapping keyboard keys to joystick axes and buttons

axes:
  # Axis mappings: Define which keyboard keys control specific joystick axes
  # Format: '<key>: [<axis_index>, <axis_value>, <mode>'
  w: [1, -1.0, 'normal']
  s: [1, 1.0, 'normal']
  a: [3, -1.0, 'normal']
  d: [3, 1.0, 'normal']
  Key.up: [1, -1.0, 'normal']
  Key.down: [1, 1.0, 'normal']
  Key.left: [3, -1.0, 'normal']
  Key.right: [3, 1.0, 'normal']

# Parameters for axis increment_rate, increment_step
parameters:
  axis_increment_rate: 0.01   # Time interval for updating axis values
  axis_increment_step: 0.01   # Step size for axis value change

buttons:
  # Button mappings: Define which keyboard keys toggle specific joystick buttons
  # Format: '<key>: <button_index>'
  '0': 0
  '1': 1
  '2': 2
  '3': 3
```
