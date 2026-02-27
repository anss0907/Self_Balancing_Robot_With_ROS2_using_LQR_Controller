#!/usr/bin/env python3
"""
LQR Balancing Controller with optional Luenberger Observer.

State:  x = [theta, theta_dot, p, p_dot]
Input:  u = tau_total [N·m]

Mode is derived from use_sim_time:
  use_sim_time=True  → simulation  (effort in N·m  → /simple_effort_controller/commands)
  use_sim_time=False → hardware    (firmware units  → /motor_torque_commands)

Observer (when use_observer=True):
  x̂_dot = A·x̂ + B·u + L·(y − C·x̂)     Euler integrated each step
  y = [theta, p]                          measured outputs
  u = -K·x̂                               control uses estimated state

CSV Logging:
  Every run resets the file, then appends each control step.
  Logs measured state, estimated state (if observer on), and control output.
"""

import os
import csv
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np


class LQRBalancingController(Node):

    def __init__(self):
        super().__init__('lqr_balancing_controller')

        # ==================== CONFIGURATION ====================

        # Derive hardware mode from use_sim_time (auto-declared by ROS2)
        self._hardware_mode = not self.get_parameter('use_sim_time').value

        # Theta offset differs between sim and hardware
        THETA_OFFSET = np.radians(0.67) if not self._hardware_mode else np.radians(-0.5)
        CONTROL_FREQ_HZ = 100.0

        # ---- LQR gains (from CompensatorDesign.py) ----
        if self._hardware_mode:
            # Hardware gains (Q=[1e6, 1e7, 200, 2000]/100) — DO NOT CHANGE
            K_THETA      = -28.873463575411467
            K_THETA_DOT  = -31.861376174875303
            K_P          = -0.14142135623731317
            K_P_DOT      = -1.481668405187033
        else:
            # Simulation gains (Q=[2e5, 2e5, 1, 10]/100) — low theta_dot to reduce oscillation
            K_THETA      = -17.272086567574714
            K_THETA_DOT  = -4.954627886272738
            K_P          = -0.010000000000000061
            K_P_DOT      = -0.2535088155456353

        # ---- Observer matrices (continuous, from CompensatorDesign.py) ----
        A_OBS = np.array([
            [ 0.               ,  1.               ,  0.               ,  0.               ],
            [62.43158530384362 ,  0.               ,  0.               ,  0.               ],
            [ 0.               ,  0.               ,  0.               ,  1.               ],
            [-2.250134743959207,  0.               ,  0.               ,  0.               ]])

        B_OBS = np.array([
            [ 0.               ],
            [-7.962258583790368],
            [ 0.               ],
            [ 0.667206760622431]])

        C_OBS = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0]])

        L_OBS = np.array([
            [ 1.029032611452213e+02, -6.773656159341603e-02],
            [ 2.945428712816812e+02, -4.696143658992688e+00],
            [-6.773656159341603e-02,  5.520870504140605e+00],
            [-2.648134212685400e+00,  1.024229968263310e+01]])

        # Motor
        KT_PER_MOTOR = 0.08    # [N·m per firmware unit per motor]
        RESPONSE_SCALE = 1.0
        WHEEL_RADIUS = 0.08255  # [m]

        # Motor imbalance compensation (left is ~50% weaker than right)
        # Scale > 1 means that motor gets a LARGER command to compensate.
        LEFT_MOTOR_SCALE  = 2.0
        RIGHT_MOTOR_SCALE = 1.0

        # Safety  (simulation can handle more torque than real hardware)
        TORQUE_MAX_FW = 40
        TORQUE_MAX_NM = 20.0 
        EMERGENCY_TILT_LIMIT = 15.0  # degrees

        # Deadband
        STABLE_THETA_THRESHOLD = np.radians(0.3) if not self._hardware_mode else np.radians(1.5)
        STABLE_THETA_DOT_THRESHOLD = np.radians(2.0) if not self._hardware_mode else np.radians(5.0)
        ENABLE_DEADBAND = True

        # Misc
        POSITION_MAX = 2.0
        THETA_DOT_FILTER_ALPHA = 0.3 if not self._hardware_mode else 0.15

        # ==================== STORE CONFIG ====================
        self.control_freq = CONTROL_FREQ_HZ
        self.control_dt = 1.0 / self.control_freq
        self.K = np.array([K_THETA, K_THETA_DOT, K_P, K_P_DOT])
        self.A_obs = A_OBS
        self.B_obs = B_OBS
        self.C_obs = C_OBS
        self.L_obs = L_OBS
        self.wheel_radius = WHEEL_RADIUS
        self.theta_offset = THETA_OFFSET
        self.Kt = KT_PER_MOTOR
        self.response_scale = RESPONSE_SCALE
        self.torque_max_fw = TORQUE_MAX_FW
        self.torque_max_nm = TORQUE_MAX_NM
        self.left_motor_scale = LEFT_MOTOR_SCALE
        self.right_motor_scale = RIGHT_MOTOR_SCALE
        self.alpha = THETA_DOT_FILTER_ALPHA
        self.stable_theta_thresh = STABLE_THETA_THRESHOLD
        self.stable_theta_dot_thresh = STABLE_THETA_DOT_THRESHOLD
        self.enable_deadband = ENABLE_DEADBAND
        self.emergency_tilt_limit = EMERGENCY_TILT_LIMIT
        self.position_max = POSITION_MAX

        # ==================== PARAMETERS ====================
        self.declare_parameter('use_observer', False)
        self.declare_parameter('is_balance_only', True)
        self.declare_parameter('csv_log_path',
                               os.path.join(os.path.expanduser('~'),
                                            'balancing_ws/src/LQR_Calculations/balancing_log.csv'))

        self.use_observer = self.get_parameter('use_observer').value
        self.is_balance_only = self.get_parameter('is_balance_only').value
        self.csv_log_path = self.get_parameter('csv_log_path').value

        self.hardware_mode = self._hardware_mode

        # ==================== STATE VARIABLES ====================
        self.theta_measured = 0.0
        self.theta_prev = 0.0
        self.theta_dot_filtered = 0.0
        self.p = 0.0
        self.v = 0.0
        self.wheel_velocities = [0.0, 0.0]
        self.wheel_positions = [0.0, 0.0]
        self.last_control_time = None
        self.last_tau_cmd = 0.0  # store for observer prediction

        # Observer state estimate (initialised from first measurement)
        self.x_hat = np.zeros(4)
        self._observer_initialized = False

        # Data freshness
        self.pitch_received = False
        self.joint_states_received = False
        self.last_pitch_time = None
        self.last_joint_states_time = None
        self.SENSOR_TIMEOUT_SEC = 0.2

        # Logging
        self.log_counter = 0
        self.log_interval = int(self.control_freq / 5)

        # ==================== CSV LOGGING ====================
        self._init_csv()

        # ==================== SUBSCRIBERS ====================
        self.pitch_sub = self.create_subscription(
            Float64, '/robot_pitch', self.pitch_callback, 10)

        # Always subscribe to /joint_states.
        # In simulation with encoder noise, remap in launch file.
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)

        # ==================== PUBLISHERS ====================
        if self.hardware_mode:
            self.motor_cmd_pub = self.create_publisher(
                Float64MultiArray, '/motor_torque_commands', 10)
        else:
            self.motor_cmd_pub = self.create_publisher(
                Float64MultiArray, '/simple_effort_controller/commands', 10)

        # ==================== CONTROL TIMER ====================
        self.control_timer = self.create_timer(self.control_dt, self.control_callback)

        # ==================== STARTUP LOG ====================
        mode_str = "HARDWARE" if self.hardware_mode else "SIMULATION"
        obs_str = "ON" if self.use_observer else "OFF"
        self.get_logger().info(
            f'LQR Controller [{mode_str}] | Observer: {obs_str} | '
            f'{self.control_freq:.0f}Hz | CSV: {self.csv_log_path}')

    # ──────────────────────────────────────────────────────────────────────
    # CSV
    # ──────────────────────────────────────────────────────────────────────
    def _init_csv(self):
        """Create / overwrite the CSV log file with a header row."""
        os.makedirs(os.path.dirname(self.csv_log_path) or '.', exist_ok=True)
        self._csv_file = open(self.csv_log_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        header = [
            'time_s',
            'theta_meas', 'theta_dot_meas', 'p_meas', 'p_dot_meas',
            'theta_est',  'theta_dot_est',  'p_est',  'p_dot_est',
            'tau_cmd',
        ]
        self._csv_writer.writerow(header)
        self._csv_start_time = None

    def _log_csv(self, t, x_meas, x_est, tau_cmd):
        if self._csv_start_time is None:
            self._csv_start_time = t
        elapsed = (t - self._csv_start_time).nanoseconds / 1e9
        row = [f'{elapsed:.4f}']
        row += [f'{v:.8f}' for v in x_meas]
        row += [f'{v:.8f}' for v in x_est]
        row += [f'{tau_cmd:.8f}']
        self._csv_writer.writerow(row)

    # ──────────────────────────────────────────────────────────────────────
    # CALLBACKS
    # ──────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        # Send zero commands and close CSV
        zero_msg = Float64MultiArray()
        zero_msg.data = [0.0, 0.0]
        self.motor_cmd_pub.publish(zero_msg)
        if hasattr(self, '_csv_file') and not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()
            self.get_logger().info(f'CSV log saved to {self.csv_log_path}')
        super().destroy_node()

    def pitch_callback(self, msg):
        self.theta_measured = msg.data
        self.pitch_received = True
        self.last_pitch_time = self.get_clock().now()

    def joint_states_callback(self, msg):
        try:
            left_idx = msg.name.index('wheel_left_joint')
            right_idx = msg.name.index('wheel_right_joint')
            self.wheel_velocities = [msg.velocity[left_idx], msg.velocity[right_idx]]
            if len(msg.position) > max(left_idx, right_idx):
                self.wheel_positions = [msg.position[left_idx], msg.position[right_idx]]
            self.joint_states_received = True
            self.last_joint_states_time = self.get_clock().now()
        except (ValueError, IndexError) as e:
            if not self.joint_states_received:
                self.get_logger().warn(f'Wheel joints not found: {e}')

    # ──────────────────────────────────────────────────────────────────────
    # CONTROL
    # ──────────────────────────────────────────────────────────────────────
    def control_callback(self):
        if not self.pitch_received or not self.joint_states_received:
            return

        current_time = self.get_clock().now()

        # ---- Sensor watchdog (hardware only) ----
        if self.hardware_mode:
            pitch_age = (current_time - self.last_pitch_time).nanoseconds / 1e9 if self.last_pitch_time else 999.0
            joint_age = (current_time - self.last_joint_states_time).nanoseconds / 1e9 if self.last_joint_states_time else 999.0
            if pitch_age > self.SENSOR_TIMEOUT_SEC or joint_age > self.SENSOR_TIMEOUT_SEC:
                zero_msg = Float64MultiArray()
                zero_msg.data = [0.0, 0.0]
                self.motor_cmd_pub.publish(zero_msg)
                self.get_logger().error(
                    f'WATCHDOG: stale data pitch={pitch_age:.3f}s joint={joint_age:.3f}s — MOTORS OFF',
                    throttle_duration_sec=0.5)
                return

        if self.last_control_time is None:
            self.last_control_time = current_time
            return

        dt = (current_time - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = current_time
        if dt <= 0 or dt > 0.1:
            return

        # ---- Measured state ----
        theta = self.theta_measured - self.theta_offset

        # Emergency tilt stop
        if abs(theta) > np.radians(self.emergency_tilt_limit):
            zero_msg = Float64MultiArray()
            zero_msg.data = [0.0, 0.0]
            self.motor_cmd_pub.publish(zero_msg)
            self.p = 0.0
            self.theta_dot_filtered = 0.0
            self.theta_prev = theta
            self.x_hat = np.zeros(4)
            self.get_logger().error(
                f'EMERGENCY STOP: {np.degrees(theta):.1f}° > ±{self.emergency_tilt_limit:.1f}°',
                throttle_duration_sec=1.0)
            return

        # theta_dot (numerical derivative + low-pass filter + hard clamp)
        theta_dot_raw = (theta - self.theta_prev) / dt
        theta_dot_raw = np.clip(theta_dot_raw, np.radians(-40.0), np.radians(40.0))
        self.theta_dot_filtered = (self.alpha * self.theta_dot_filtered +
                                   (1.0 - self.alpha) * theta_dot_raw)
        self.theta_dot_filtered = np.clip(self.theta_dot_filtered,
                                          np.radians(-40.0), np.radians(40.0))
        self.theta_prev = theta

        # Wheel-based forward velocity & position
        omega_avg = (self.wheel_velocities[0] + self.wheel_velocities[1]) / 2.0
        p_dot = omega_avg * self.wheel_radius

        if self.hardware_mode:
            pos_avg = (self.wheel_positions[0] + self.wheel_positions[1]) / 2.0
            p = pos_avg * self.wheel_radius
        else:
            p = self.p + p_dot * dt

        p = np.clip(p, -self.position_max, self.position_max)
        self.p = p

        # Measured state vector
        x_meas = np.array([theta, self.theta_dot_filtered, p, p_dot])

        # ---- Observer (passive Euler integration — monitoring only) ----
        # Control ALWAYS uses measured states.  The observer runs alongside
        # to produce x_hat for logging / visualisation comparison.
        if self.use_observer:
            # Seed observer from first measurement so it doesn't start at zero
            if not self._observer_initialized:
                self.x_hat = x_meas.copy()
                self._observer_initialized = True

            y_meas = np.array([theta, p])              # measured outputs
            y_hat = self.C_obs @ self.x_hat             # predicted outputs
            innovation = y_meas - y_hat

            x_hat_dot = (self.A_obs @ self.x_hat
                         + (self.B_obs @ np.array([self.last_tau_cmd])).flatten()
                         + (self.L_obs @ innovation))
            self.x_hat += dt * x_hat_dot
        else:
            self.x_hat = x_meas.copy()  # keep in sync for CSV

        # Control always uses direct sensor measurements
        x_ctrl = x_meas.copy()

        # ---- Deadband ----
        is_stable = False
        if self.enable_deadband:
            is_stable = (abs(x_ctrl[0]) < self.stable_theta_thresh and
                         abs(x_ctrl[1]) < self.stable_theta_dot_thresh)

        if is_stable:
            tau_cmd = 0.0
            tau_raw = 0.0
        else:
            # u = -K · x
            tau_raw = float(-self.K @ x_ctrl) * self.response_scale
            tau_cmd = np.clip(tau_raw, -self.torque_max_nm, self.torque_max_nm)

        self.last_tau_cmd = tau_cmd
        is_saturated = abs(tau_raw) > self.torque_max_nm

        # ---- Publish command ----
        cmd_msg = Float64MultiArray()
        if self.hardware_mode:
            fw_base = tau_cmd / (2.0 * self.Kt)
            fw_left  = np.clip(fw_base * self.left_motor_scale,
                               -self.torque_max_fw, self.torque_max_fw)
            fw_right = np.clip(fw_base * self.right_motor_scale,
                               -self.torque_max_fw, self.torque_max_fw)
            cmd_msg.data = [float(fw_left), float(fw_right)]
        else:
            half_tau = tau_cmd / 2.0
            cmd_msg.data = [float(half_tau), float(half_tau)]
        self.motor_cmd_pub.publish(cmd_msg)

        # ---- CSV log (every step) ----
        self._log_csv(current_time, x_meas, self.x_hat, tau_cmd)

        # ---- Console log (5 Hz) ----
        self.log_counter += 1
        if self.log_counter >= self.log_interval:
            self.log_counter = 0
            sat = ' [SAT]' if is_saturated else ''
            obs = ' [OBS]' if self.use_observer else ''
            self.get_logger().info(
                f'θ={np.degrees(x_ctrl[0]):6.2f}° θ̇={np.degrees(x_ctrl[1]):6.2f}°/s '
                f'p={x_ctrl[2]:6.3f}m ṗ={x_ctrl[3]:5.3f}m/s τ={tau_cmd:.3f}Nm{sat}{obs}')


def main(args=None):
    rclpy.init(args=args)
    node = LQRBalancingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down LQR Balancing Controller')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
