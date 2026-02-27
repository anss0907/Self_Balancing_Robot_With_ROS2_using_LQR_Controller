#ifndef GRACE_INTERFACE_HPP
#define GRACE_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <grace_msg/msg/motor_feedback.hpp>

#include <vector>
#include <string>
#include <sstream>


namespace grace_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class GraceInterface : public hardware_interface::SystemInterface
{
public:
  GraceInterface();
  virtual ~GraceInterface();

  // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // Implementing hardware_interface::SystemInterface
  CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // ESP32 data structure (updated for torque-control firmware)
  struct ESP32Data
  {
    uint32_t esp32_dt_ms = 0;          // T field: ms since last send (from ESP32)
    uint8_t driver_status = 0;         // D field: hoverboard driver responding
    int16_t battery_mv = 0;
    int16_t temperature_deci_c = 0;
    int16_t target_torque_left = 0;    // TL field: echo of left torque command
    int16_t target_torque_right = 0;   // TR field: echo of right torque command
    int16_t left_measured_rpm = 0;     // Lm field: measured speed (RPM)
    int16_t right_measured_rpm = 0;    // Rm field: measured speed (RPM)
    int16_t curL_DC = 0;               // cL field: left DC bus current (mA)
    int16_t curR_DC = 0;               // cR field: right DC bus current (mA)
    int16_t iqL = 0;                   // qL field: left q-axis current (mA)
    int16_t iqR = 0;                   // qR field: right q-axis current (mA)
    int16_t idL = 0;                   // dL field: left d-axis current (mA)
    int16_t idR = 0;                   // dR field: right d-axis current (mA)
    int16_t angleL = 0;                // aL field: left motor electrical angle
    int16_t angleR = 0;                // aR field: right motor electrical angle
    uint8_t imu1_status = 0;
    int16_t ax1 = 0, ay1 = 0, az1 = 0;
    int16_t gx1 = 0, gy1 = 0, gz1 = 0;
    uint8_t imu2_status = 0;
    int16_t ax2 = 0, ay2 = 0, az2 = 0;
    int16_t gx2 = 0, gy2 = 0, gz2 = 0;
    bool valid = false;
  };

  // Parsing helpers
  ESP32Data parseESP32Line(const std::string& line);
  int16_t parseField(const std::string& line, const std::string& label, int digits);
  uint32_t parseDtField(const std::string& line, const std::string& label);
  uint8_t parseStatusField(const std::string& line, const std::string& label);

  // Conversion helpers
  double rawAccelToMps2(int16_t raw_value);
  double rawGyroToRadps(int16_t raw_value);
  double rpmToRadps(int16_t rpm);

  // Publishing helpers
  void publishIMU1(const ESP32Data& data);
  void publishIMU2(const ESP32Data& data);
  void publishBattery(const ESP32Data& data);
  void publishTemperature(const ESP32Data& data);
  void publishJointStates(const ESP32Data& data);
  void publishMotorFeedback(const ESP32Data& data);

  // Direct torque command callback (firmware units, [-1000, 1000] per wheel)
  void motorTorqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // Serial
  LibSerial::SerialPort esp32_;
  std::string port_;
  std::string read_buffer_;

  // Command source mode
  std::string command_source_;  // "ros2_control" or "direct"
  std::vector<double> direct_torque_commands_;  // [left_torque, right_torque] in firmware units

  // ros2_control state/command vectors
  std::vector<double> effort_commands_;     // torque commands [N·m] from ros2_control (balancing)
  std::vector<double> velocity_commands_;   // velocity commands [rad/s] from ros2_control (diff_drive compat)
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;

  // ROS2 Node and Publishers
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<grace_msg::msg::MotorFeedback>::SharedPtr motor_feedback_pub_;

  // Direct torque subscriber (for bypassing ros2_control)
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motor_torque_sub_;

  // Conversion constants
  static constexpr double ACCEL_SCALE = 16384.0;  // MPU6050 ±2g range
  static constexpr double GYRO_SCALE = 131.0;     // MPU6050 ±250°/s range
  static constexpr double G_TO_MPS2 = 9.81;
  static constexpr double DEG_TO_RAD = M_PI / 180.0;
  static constexpr double RPM_TO_RADPS = 2.0 * M_PI / 60.0;

  // Torque conversion: physical N·m = Kt * firmware_unit (per motor)
  // CALIBRATED 2026-02-20 using kt_calibration_node with floor resistance
  static constexpr double KT_PER_MOTOR = 0.001162;  // [N·m per firmware unit]
  static constexpr int16_t TORQUE_MAX_FW = 1000;  // firmware clamp
};
}  // namespace grace_firmware


#endif  // GRACE_INTERFACE_HPP