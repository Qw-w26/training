import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rm_msgs.msg import WheelLegCmd
from sensor_msgs.msg import Imu, JointState, Joy

from .lqr_update import lqr_controller
from .vmc_cal import five_links_param


class BaseController:
    """Controller 通用基类"""


class PIDController(BaseController):
    def __init__(self, kp, ki, kd, max_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, current, target):
        error = target - current
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.max_out, min(self.max_out, output))


class RollLegController(BaseController):
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, max_out=0.14,
                 min_leg=0.10, max_leg=0.14):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.min_leg = min_leg
        self.max_leg = max_leg
        self.integral = 0.0
        self.prev_error = 0.0

    def compute_leg_lengths(self, current_roll, target_roll, base_length):
        error = target_roll - current_roll
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        delta = self.kp * error + self.ki * self.integral + self.kd * derivative
        delta = max(-self.max_out, min(self.max_out, delta))
        left = max(self.min_leg, min(self.max_leg, base_length - delta))
        right = max(self.min_leg, min(self.max_leg, base_length + delta))
        return left, right


class WheelSpeedPIController(BaseController):
    def __init__(self, kp=0.25, ki=0.20, max_output=0.45):
        self.kp = kp
        self.ki = ki
        self.max_output = max_output
        self.integral = 0.0

    def compute(self, target_speed, measured_speed, dt):
        error = target_speed - measured_speed
        candidate_integral = self.integral + error * dt
        raw_output = self.kp * error + self.ki * candidate_integral
        output = max(-self.max_output, min(self.max_output, raw_output))
        if (raw_output == output
                or (raw_output > self.max_output and error < 0.0)
                or (raw_output < -self.max_output and error > 0.0)):
            self.integral = candidate_integral
        return output


class WheelLegController(BaseController, Node):
    def __init__(self):
        Node.__init__(self, 'wheel_leg_controller')
        self.declare_parameter('dt', 0.002)
        self.declare_parameter('wheel_radius', 0.06)
        self.declare_parameter('target_length', 0.10)
        self.declare_parameter('joy_timeout', 0.5)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('max_linear_speed', 0.6)
        self.declare_parameter('max_yaw_rate', 2.0)
        self.declare_parameter('joy_linear_axis', 1)
        self.declare_parameter('joy_yaw_axis', 3)
        self.declare_parameter('joy_linear_sign', -1.0)
        self.declare_parameter('joy_yaw_sign', -1.0)
        self.declare_parameter('wheel_pi_only', False)
        self.declare_parameter('wheel_speed_kp', 0.25)
        self.declare_parameter('wheel_speed_ki', 0.20)
        self.declare_parameter('wheel_speed_pi_limit', 0.0)
        self.declare_parameter('lqr_velocity_scale', 0.5)
        self.dt = float(self.get_parameter('dt').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.target_length = float(self.get_parameter('target_length').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.joy_linear_axis = int(self.get_parameter('joy_linear_axis').value)
        self.joy_yaw_axis = int(self.get_parameter('joy_yaw_axis').value)
        self.joy_linear_sign = float(self.get_parameter('joy_linear_sign').value)
        self.joy_yaw_sign = float(self.get_parameter('joy_yaw_sign').value)
        self.lqr_velocity_scale = float(self.get_parameter('lqr_velocity_scale').value)

        self.right_leg = five_links_param(dt=self.dt)
        self.left_leg = five_links_param(dt=self.dt)
        self.right_lqr = lqr_controller(dt=self.dt)
        self.left_lqr = lqr_controller(dt=self.dt)
        self.right_length_pid = PIDController(700.0, 0.0, 5.0, 150.0)
        self.left_length_pid = PIDController(700.0, 0.0, 5.0, 150.0)
        self.right_yaw_pid = PIDController(0.5, 0.0, 1.0, 2.0)
        self.left_yaw_pid = PIDController(0.5, 0.0, 1.0, 2.0)
        self.roll_balancer = RollLegController(
            kp=1.0, ki=0.0, kd=0.0, max_out=0.14,
            min_leg=0.10, max_leg=0.14)
        self.wheel_pi_only = bool(self.get_parameter('wheel_pi_only').value)
        self.wheel_speed_pi = WheelSpeedPIController(
            kp=float(self.get_parameter('wheel_speed_kp').value),
            ki=float(self.get_parameter('wheel_speed_ki').value),
            max_output=float(self.get_parameter('wheel_speed_pi_limit').value))

        self.imu = None
        self.joints = None
        self.last_stamp = None
        self.current_v = 0.0
        self.target_v = 0.0
        self.target_w = 0.0
        self.cmd_vel_target = (0.0, 0.0)
        self.joy_target = (0.0, 0.0)
        self.last_cmd_vel_time = None
        self.last_joy_time = None
        self.previous_joy_buttons = (0, 0)
        self.cmd_pub = self.create_publisher(WheelLegCmd, 'actuator_cmd', 10)
        self.create_subscription(Imu, 'imu', self.on_imu, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.get_logger().info('LQR/VMC controller ready; waiting for /imu and /joint_states')

    # Callback 函数们
    def on_cmd_vel(self, msg):
        self.cmd_vel_target = (float(msg.linear.x), float(msg.angular.z))
        self.last_cmd_vel_time = time.monotonic()

    def on_joy(self, msg):
        """处理手柄输入"""
        now = time.monotonic()
        self.last_joy_time = now
        axes = msg.axes
        # 获取前后转向轴
        linear_axis = (axes[self.joy_linear_axis]
                       if len(axes) > self.joy_linear_axis else 0.0)
        # 获取左右转向轴
        yaw_axis = (axes[self.joy_yaw_axis]
                    if len(axes) > self.joy_yaw_axis else 0.0)
        # 防误触的死区处理
        if abs(linear_axis) < 0.1:
            linear_axis = 0.0
        if abs(yaw_axis) < 0.1:
            yaw_axis = 0.0
        # 摇杆输入转换为机器人速度目标
        self.joy_target = (
            self.joy_linear_sign * linear_axis * self.max_linear_speed,
            self.joy_yaw_sign * yaw_axis * self.max_yaw_rate,
        )

        buttons = msg.buttons
        # buttons[4]：LB，左肩键
        # buttons[5]：RB，右肩键
        lb = int(buttons[4]) if len(buttons) > 4 else 0
        rb = int(buttons[5]) if len(buttons) > 5 else 0
        old_lb, old_rb = self.previous_joy_buttons
        if rb and not old_rb:
            self.target_length = min(0.14, self.target_length + 0.005)
        if lb and not old_lb:
            self.target_length = max(0.10, self.target_length - 0.005)
        self.previous_joy_buttons = (lb, rb)

    def active_targets(self):
        """选择输入源"""
        now = time.monotonic()
        if self.last_joy_time is not None and now - self.last_joy_time <= self.joy_timeout:
            return self.joy_target
        if (self.last_cmd_vel_time is not None
                and now - self.last_cmd_vel_time <= self.cmd_vel_timeout):
            return self.cmd_vel_target
        return 0.0, 0.0

    def on_imu(self, msg):
        self.imu = msg
        self.compute_if_ready()

    def on_joints(self, msg):
        self.joints = msg
        self.compute_if_ready()

    @staticmethod
    def stamp_key(msg):
        return msg.header.stamp.sec, msg.header.stamp.nanosec

    def compute_if_ready(self):
        if self.imu is None or self.joints is None:
            return
        joint_stamp = self.stamp_key(self.joints)
        # IMU 和关节消息的时间戳不一致或这组 message 计算过控制量，则直接返回
        if self.stamp_key(self.imu) != joint_stamp or joint_stamp == self.last_stamp:
            return
        control_dt = self.dt
        if self.last_stamp is not None:
            previous_ns = self.last_stamp[0] * 1_000_000_000 + self.last_stamp[1]
            current_ns = joint_stamp[0] * 1_000_000_000 + joint_stamp[1]
            elapsed = (current_ns - previous_ns) / 1_000_000_000.0
            if elapsed > 0.0:
                control_dt = elapsed
        self.last_stamp = joint_stamp
        try:
            position = dict(zip(self.joints.name, self.joints.position))
            velocity = dict(zip(self.joints.name, self.joints.velocity))
            required = ('right1', 'right2', 'left1', 'left2', 'rightwheel', 'leftwheel')
            if any(name not in position or name not in velocity for name in required):
                raise ValueError('joint_states is missing one or more required joints')
            self.compute_control(position, velocity, control_dt)
        except (ValueError, KeyError, ZeroDivisionError) as exc:
            self.get_logger().error(f'Cannot compute control: {exc}')

    def compute_control(self, pos, vel, dt):
        """
        接收：
            - pos：六个关节的位置。
            - vel：六个关节的速度。
        主要执行以下操作：
            1. 姿态解算。
            2. 计算轮子运动状态。
            3. 生成平滑速度目标。
            4. 更新 LQR 目标。
            5. 计算左右腿目标长度。
            6. 分别调用左右腿控制器。
            7. 发布最终电机指令。
        """
        q = self.imu.orientation
        w, x, y, z = q.w, q.x, q.y, q.z
        sin_pitch = 2.0 * (w * y - z * x)
        pitch = -math.asin(max(-1.0, min(1.0, sin_pitch)))
        roll_sin = 2.0 * (w * x + y * z)
        roll_cos = 1.0 - 2.0 * (x * x + y * y)
        roll = -math.atan2(roll_sin, roll_cos)
        gyro_pitch = -float(self.imu.angular_velocity.y)
        gyro_yaw = float(self.imu.angular_velocity.x)

        common_speed = (vel['leftwheel'] - vel['rightwheel']) / 2.0
        common_pos = (pos['leftwheel'] - pos['rightwheel']) / 2.0
        self.target_v, self.target_w = self.active_targets()
        self.current_v = self.ramp_velocity(self.current_v, self.target_v, dt)
        if not (self.current_v == 0.0 and self.target_v == 0.0):
            self.right_lqr.x_b_set = common_pos * self.wheel_radius
            self.left_lqr.x_b_set = common_pos * self.wheel_radius
        # The source LQR uses the opposite wheel sign from ROS linear.x.
        lqr_v_set = -self.current_v
        if self.wheel_pi_only:
            pi_torque = self.wheel_speed_pi.compute(
                -self.current_v, common_speed * self.wheel_radius, dt)
            cmd = WheelLegCmd()
            cmd.right_wheel = -pi_torque
            cmd.left_wheel = pi_torque
            self.cmd_pub.publish(cmd)
            return

        lqr_v_set *= self.lqr_velocity_scale
        self.right_lqr.v_set = lqr_v_set
        self.left_lqr.v_set = lqr_v_set

        target_left, target_right = self.roll_balancer.compute_leg_lengths(
            current_roll=roll, target_roll=0.0, base_length=self.target_length)

        right = self.leg_control(
            0, self.right_leg, self.right_lqr, self.right_length_pid,
            self.right_yaw_pid, pos['right1'], pos['right2'], pitch,
            gyro_pitch, gyro_yaw, common_pos, common_speed, target_right)
        left = self.leg_control(
            1, self.left_leg, self.left_lqr, self.left_length_pid,
            self.left_yaw_pid, pos['left1'], pos['left2'], pitch,
            gyro_pitch, gyro_yaw, common_pos, common_speed, target_left)

        pi_torque = self.wheel_speed_pi.compute(
            lqr_v_set, common_speed * self.wheel_radius, dt)
        right = (right[0], right[1], max(-2.0, min(2.0, right[2] - pi_torque)))
        left = (left[0], left[1], max(-2.0, min(2.0, left[2] + pi_torque)))

        # 发布最终电机指令
        cmd = WheelLegCmd()
        cmd.right_phi1, cmd.right_phi4, cmd.right_wheel = right
        cmd.left_phi1, cmd.left_phi4, cmd.left_wheel = left
        self.cmd_pub.publish(cmd)

    @staticmethod
    def ramp_velocity(current, target, dt):
        """让加速时速度变化较慢，减速或反向时速度变化较快"""
        rate = 0.8 if abs(target) > abs(current) and target * current >= 0 else 1.4
        step = max(-dt * rate, min(dt * rate, target - current))
        result = current + step
        return 0.0 if abs(result) < 0.001 and target == 0.0 else result

    def leg_control(self, side, leg, lqr, length_pid, yaw_pid, motor1, motor2,
                    pitch, gyro_pitch, gyro_yaw, common_pos, common_speed,
                    target_length):
        """
        完成单条腿的全部控制计算。
        五个步骤：
            1. 五连杆正运动学。
            2. 更新 LQR 状态。
            3. 计算 LQR 增益。
            4. 计算腿长控制和姿态控制。
            5. 通过 VMC 转换成关节力矩。
        """
        leg.forward_kinematics_cal(side, motor1, motor2, pitch, gyro_pitch)
        # The right-leg LQR applies its own wheel-axis sign inversion too.
        lqr_pos = -common_pos if side == 0 else common_pos
        lqr_speed = -common_speed if side == 0 else common_speed
        lqr.states_update(side, leg, pitch, gyro_pitch, lqr_pos, lqr_speed,
                          self.wheel_radius)
        lqr.calc_k_matrix_from_poly(leg.L_0)

        # 来自 02-ros2-wheelleg/Wheel_Leg/LQR_VMC/robot_control_main.py
        # VMC 虚拟力基本计算
        f0_ctrl = length_pid.compute(leg.L_0, target_length)
        cosine = math.cos(leg.theta)
        f0 = (4.0 / cosine if abs(cosine) > 1e-9 else 0.0) + f0_ctrl
        f0 = max(-100.0, min(100.0, f0))
        # tp
        tp = sum(lqr.K[i] * value for i, value in (
            (1, lqr.theta), (3, lqr.d_theta), (5, lqr.x_b - lqr.x_b_set),
            (7, lqr.v_b_whole - lqr.v_set), (9, lqr.phi), (11, lqr.d_phi)))
        # wheel
        wheel = sum(lqr.K[i] * value for i, value in (
            (0, lqr.theta), (2, lqr.d_theta), (4, lqr.x_b - lqr.x_b_set),
            (6, lqr.v_b_whole - lqr.v_set), (8, lqr.phi), (10, lqr.d_phi)))
        yaw = yaw_pid.compute(gyro_yaw, self.target_w)
        wheel = wheel - yaw if side == 0 else -wheel - yaw
        wheel = max(-1.5, min(1.5, wheel))
        t1, t2 = leg.VMC_torque_cal(f0, tp)
        if side == 0:
            return t1, t2, wheel
        return -t1, -t2, wheel
