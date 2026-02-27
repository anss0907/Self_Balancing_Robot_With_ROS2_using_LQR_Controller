# Motor feedback from hoverboard driver via ESP32
# All data echoed back from ESP32 firmware at ~200Hz

std_msgs/Header header

# Target torque commands (echo of what was sent to motors)
float64 target_torque_left      # firmware units [-1000, 1000]
float64 target_torque_right     # firmware units [-1000, 1000]

# Measured currents (directly from FOC controller)
float64 iq_left                 # q-axis current (torque-producing) [A]
float64 iq_right                # q-axis current (torque-producing) [A]
float64 id_left                 # d-axis current (flux) [A]
float64 id_right                # d-axis current (flux) [A]
float64 current_dc_left         # DC bus current [A]
float64 current_dc_right        # DC bus current [A]

# Measured torque (calculated from iq)
# torque = motor_constant × iq
float64 measured_torque_left    # [N·m]
float64 measured_torque_right   # [N·m]

# Speed feedback
float64 rpm_left                # [RPM]
float64 rpm_right               # [RPM]
float64 velocity_left           # [rad/s]
float64 velocity_right          # [rad/s]

# Motor electrical angle (for debugging FOC)
float64 electrical_angle_left   # [degrees, 0-360]
float64 electrical_angle_right  # [degrees, 0-360]
