# wheel_leg_controller

This is a ROS 2 control node responsible for receiving data from the `/imu` and `/joint_states` topics, calculating LQR and VMC control torques, and publishing control commands of type `rm_msgs/msg/WheelLegCmd` on the `/actuator_cmd` topic.

The node does not contain any MuJoCo imports or simulation code. The simulator continues to run independently through the `wheel_leg_sim` node.

## Control

The controller uses the following parameters:

- Control period: `dt = 0.002 s`.
- Wheel radius: `0.06 m`.
- Nominal leg length: `0.10 m`.

The 12-element LQR gain is calculated using the cubic polynomial coefficients in `lqr_update.py`. This file is copied from the original `Wheel_Leg/LQR_VMC/lqr_update.py`.

These coefficients originate from the LQR fitting results in the MATLAB file `cal_lqr_k.m`.

The five-link mechanism's forward kinematics and Jacobian-based torque mapping algorithms come from the original `vmc_cal.py`.

Each leg uses the original leg-length PID parameters:

- Proportional gain P: `700`.
- Integral gain I: `0`.
- Derivative gain D: `5`.

Pitch angle and wheel state feedback control follow the implementation in `robot_control_main.py`.

The standing position target is updated while the robot is moving. When the requested velocity is zero, the position target remains unchanged.

The roll controller adjusts the left and right target leg lengths independently around the base leg length.

The following controller classes are all implemented in `controller.py`:

- `BaseController`: Base controller.
- `PIDController`: PID controller.
- `RollLegController`: Roll leg controller.
- `WheelLegController`: Wheel-leg robot controller.

The three specific controller classes inherit from the common base class `BaseController`.

### Speed Control and Feedback

`/cmd_vel.linear.x` is used to set the target velocity.

The controller applies the following velocity ramp limits:

- Acceleration rate: `0.8 m/s²`.
- Deceleration rate: `1.4 m/s²`.

The velocity ramp is calculated using the actual elapsed time between paired state messages. Therefore, the velocity change at each update depends on the actual controller update interval.

The wheel rotation direction in the original LQR controller is opposite to the direction defined by ROS `linear.x`.

Therefore, the controller reverses the wheel direction sign and applies the empirically measured parameter:

`lqr_velocity_scale = 0.5`

This parameter is used to match the commanded velocity with the measured actual velocity.

`/cmd_vel.angular.z` sets the target value for the yaw PID controller.

The controller also provides an optional wheel speed PI control function.

By default, this function is disabled to retain the full LQR balance control. This additional control function can be enabled through the `wheel_speed_pi_limit` parameter.

## Gamepad and Keyboard Control

Both gamepad and keyboard input methods publish messages of type `sensor_msgs/msg/Joy` on the `/joy` topic.

**The controller itself does not depend on the `keyboard_joy` package.**

The default Xbox gamepad layout uses the following axis mappings:

- `axes[1]`: Forward and backward movement.
- `axes[3]`: Yaw control (turning).

The controller uses the conventional negative-axis sign convention.

The axis indices and direction signs can be configured through the following parameters:

| Parameter | Description |
|---|---|
| `joy_linear_axis` | Specifies the input axis index for linear velocity |
| `joy_yaw_axis` | Specifies the input axis index for yaw control |
| `joy_linear_sign` | Sets the direction sign for linear velocity input |
| `joy_yaw_sign` | Sets the direction sign for yaw input |

When a new `/joy` message arrives, the controller gives priority to gamepad or keyboard input.

If no new joystick message is received for longer than the time specified by `joy_timeout`, the controller switches back to using the `/cmd_vel` topic.

The default timeout is:

`joy_timeout = 0.5 s`

The controller applies the following input constraints:

- Input deadzone: `0.1`.
- Speed limits.
- Acceleration ramp limits.

The target leg length can also be adjusted using gamepad buttons:

- Button `5`: Increases the target leg length by `5 mm`.
- Button `4`: Decreases the target leg length by `5 mm`.

The allowed target leg length range is:

`0.10–0.14 m`

### 1. Using a Physical Gamepad

If using a physical gamepad, install and run the standard ROS joystick driver.

This method does not require building the `keyboard_joy` package.

#### Step 1: Install the ROS joystick driver

```bash
sudo apt install ros-${ROS_DISTRO}-joy
```

#### Step 2: Enumerate and check gamepad devices

```bash
ros2 run joy joy_enumerate_devices
```

This command enumerates the gamepad devices detected by the system.

#### Step 3: Start the joystick driver node

```bash
ros2 run joy joy_node
```

After startup, the driver node converts physical gamepad input into `/joy` messages for the controller.

### 2. Using Keyboard Control

If using keyboard input, first install `pynput` in the currently active Python environment, then build the optional `keyboard_joy` package.

#### Step 1: Enter the ROS 2 workspace

```bash
cd Tutorial/ros2_ws
```

#### Step 2: Build the keyboard control package

```bash
colcon build --packages-select keyboard_joy
source install/setup.bash
```

#### Step 3: Start the keyboard control node

```bash
ros2 run keyboard_joy joy_node
```

After startup, use the following keys to control the robot:

- Arrow keys or WASD: Control robot movement.
- `5`: Increase the target leg length.
- `4`: Decrease the target leg length.

The keyboard node reads key presses directly from the interactive terminal in which it was started. Keep that terminal focused and press the keys there. You can run `ros2 topic echo /joy` in another terminal to observe changes in the joystick axis values.

**Note: Only one `/joy` input source should run at a time.**

In other words, if you are using a physical gamepad, do not start the keyboard control node at the same time. Otherwise, the two input sources may interfere with each other.

## Run

First, make sure that the ROS 2 environment and Python environment are correctly activated.

Enter the `Tutorial/ros2_ws` workspace and run the following commands to build the controller and its dependencies:

```bash
colcon build --symlink-install --packages-up-to wheel_leg_controller
source install/setup.bash
```

### Terminal 1: Start the MuJoCo Simulator

Run:

```bash
ros2 run wheel_leg_sim wheel_leg_sim --ros-args -p viewer:=false
```

Where:

- `wheel_leg_sim`: The ROS 2 node responsible for running the MuJoCo simulation.
- `viewer:=false`: Disables the MuJoCo graphical viewer.

### Terminal 2: Start the Wheel-Leg Robot Controller

In another terminal with the same workspace environment sourced, run:

```bash
ros2 run wheel_leg_controller wheel_leg_controller
```

### Terminal 3: Start One Input Node

Choose one of the following methods according to the input device being used.

**Method A: Using a Physical Gamepad**

```bash
ros2 run joy joy_node
```

**Method B: Using Keyboard Control**

First, make sure that the optional `keyboard_joy` package has been built, then run:

```bash
ros2 run keyboard_joy joy_node
```

### Controlling the Robot Using `/cmd_vel`

If neither a keyboard nor a gamepad is used, the robot can also be controlled through the `/cmd_vel` topic.

The message type is:

`geometry_msgs/msg/Twist`

The controller uses `/cmd_vel` as its control input whenever no fresh `/joy` input has been received.

## Balance Result

Test environment:

- ROS 2 version: Jazzy.
- MuJoCo version: 3.17.0.
- Simulation viewer: Disabled (`viewer:=false`).
- Real-time simulation: Enabled (`realtime:=true`).
- Simulation speed: Real-time speed, 1x.

During the same test run, the robot completed the following stages in sequence:

1. Maintained balance in place for 5 seconds.
2. Moved at a commanded velocity of `0.3 m/s` for 12 seconds.
3. Stopped and gradually settled during the following 5 seconds.

Throughout the entire test, the robot's pitch angle remained below 10°.

### Test Results for Each Stage

| Stage | Test Description | Result |
|---|---|---|
| Stage 1: ROS communication link | Checked the data path from `/joint_states` to `/actuator_cmd` and confirmed that the simulator ran at 1x real-time speed | Passed |
| Stage 2: Wheel PI control | Set `wheel_pi_only:=true`, `wheel_speed_kp:=6`, `wheel_speed_ki:=0`, and `wheel_speed_pi_limit:=1`, with a target speed of `0.5 m/s` | Mean speed: `0.413 m/s`; error: `17.4%`. Passed, with error below 20% |
| Stage 3: VMC control | Calculated the average leg length of both legs over the final 5 seconds, with a target leg length of `0.10 m` | Mean leg length: `0.09849–0.09869 m`; maximum error: `1.51 mm`. Passed, within ±5 mm |
| Stage 4: Balance control | Measured the stationary balance state during the first 5 seconds | Maximum absolute pitch angle: `0.116°`. Passed |
| Stage 5: Speed control and recovery | Moved at `0.3 m/s` for 12 seconds, then stopped | Odometry distance: `3.686 m`; mean speed: `0.307 m/s`; peak absolute pitch angle: `1.004°`. Passed, with more than 3 m traveled without falling |
| Stage 5: Disturbance test | Applied a differential torque of `2 Nm` to the left and right wheels for `0.1 s` while the robot was balanced | Peak absolute pitch angle: `6.15°`; recovered to `1.81°` by 5.9 seconds |
| Keyboard input test | Simulated pressing the Up key through `keyboard_joy` | `/joy` successfully reached the controller; the target speed increased while the key was held and returned to zero after release |
| Gamepad input test | Used the standard Xbox gamepad axis layout on `/joy` | No physical device was available; not tested |

### Pitch Angle Changes During the 22-Second Test

The following data represents the mean pitch angle measured in each one-second interval during a 22-second standing, walking, and stopping test. The unit is degrees.

```csv
second,mean_pitch_deg
0,-0.114
1,-0.107
2,-0.106
3,-0.108
4,-0.115
5,0.539
6,0.729
7,0.921
8,0.978
9,0.996
10,1.001
11,1.003
12,1.003
13,1.004
14,1.004
15,1.004
16,1.004
17,0.290
18,-0.094
19,-0.251
20,-0.189
21,-0.106
```

The data shows that:

- Seconds 0–4: The robot remains stationary, with the mean pitch angle close to zero.
- Second 5: The robot starts moving, and the pitch angle begins to change.
- Seconds 6–16: The robot is in the movement phase, and the mean pitch angle gradually stabilizes at approximately `1.004°`.
- Second 17: After the robot stops, the pitch angle begins to decrease.
- Seconds 18–21: The robot gradually returns to a balance state close to its initial condition.

### Test Result Summary

The test results show that, under the specified simulation environment and parameter configuration:

1. The ROS 2 communication link works correctly.
2. Wheel PI control achieves speed tracking within the specified test criteria.
3. VMC leg-length control keeps the leg-length error within the required range.
4. LQR balance control maintains robot stability in both stationary and moving states.
5. After a brief external disturbance, the robot gradually recovers its balance.
6. The keyboard input pipeline has been verified through simulated key presses.
7. The gamepad axis mapping has been defined in the configuration.

## Simulation Recording

A **15-second MuJoCo GUI simulation recording** has been shared at:

[15-second MuJoCo GUI simulation recording](https://kdocs.cn/view/media/l/csgsNyp0Sdac)
