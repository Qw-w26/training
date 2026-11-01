# wheel_leg_controller

This ROS 2 node receives `/imu` and `/joint_states`, calculates LQR and VMC
torques, and publishes `rm_msgs/msg/WheelLegCmd` on `/actuator_cmd`. It contains
no MuJoCo imports or simulation code. The simulator remains the independent
`wheel_leg_sim` node.

## Control

The controller uses `dt=0.002 s`, wheel radius `0.06 m`, and nominal leg length
`0.10 m`. The 12-element LQR gain is evaluated from the cubic coefficients in
`lqr_update.py`, copied from `Wheel_Leg/LQR_VMC/lqr_update.py`. Those
coefficients originate from `cal_lqr_k.m`'s MATLAB LQR fit. Five-link forward
kinematics and Jacobian torque mapping come from the original `vmc_cal.py`.
Each leg has the original length PID gains (700, 0, 5); pitch and wheel state
feedback follows `robot_control_main.py`. The standing position target is
updated while moving and held when the requested speed is zero.
The roll controller adjusts the left and right target leg lengths around the
base length. `BaseController`, `PIDController`, `RollLegController`, and
`WheelLegController` are implemented together in `controller.py`. The three
specific controller classes inherit the common `BaseController` type.

`/cmd_vel.linear.x` sets LQR `v_set`, with a 0.8 m/s^2 acceleration and
1.4 m/s^2 deceleration ramp. The ramp uses elapsed time between paired state
messages, so its per-call change follows the actual controller update interval.
`/cmd_vel.angular.z` sets the yaw PID target.

## Run

Build in `Tutorial/ros2_ws` with the Humble environment active:

```bash
colcon build --symlink-install --packages-up-to wheel_leg_controller
source install/setup.bash
ros2 run wheel_leg_sim wheel_leg_sim --ros-args -p viewer:=false
```

In a second sourced terminal run:

```bash
ros2 run wheel_leg_controller wheel_leg_controller
```

To set a walking target, publish `geometry_msgs/msg/Twist` on `/cmd_vel`.

## Balance result

Measured with ROS 2 Jazzy and MuJoCo 3.17.0, using `viewer:=false` and
`realtime:=true` at 1x. The closed loop ran for about 16.6 simulated seconds.
The initial transient stayed within 4.92 degrees; during the final 12.1 seconds,
pitch stayed within 0.116 degrees of zero. This was a local verification run;
no recording or rosbag was saved.

- Balance duration: about 16.6 s
- Maximum absolute pitch during the initial transient: 4.92 degrees
- Maximum absolute pitch during the final 12.1 s: 0.116 degrees
- Recording or rosbag link: not recorded
