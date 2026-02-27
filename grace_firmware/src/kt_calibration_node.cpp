#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <grace_msg/msg/motor_feedback.hpp>
#include <chrono>
#include <vector>
#include <numeric>
#include <cmath>

using namespace std::chrono_literals;

class KtCalibrationNode : public rclcpp::Node
{
public:
  KtCalibrationNode() : Node("kt_calibration_node")
  {
    // Publishers and subscribers
    motor_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/motor_torque_commands", 10);
    
    motor_feedback_sub_ = this->create_subscription<grace_msg::msg::MotorFeedback>(
      "/motor_feedback", 10,
      std::bind(&KtCalibrationNode::feedbackCallback, this, std::placeholders::_1));
    
    // Wait for connections
    rclcpp::sleep_for(2s);
    
    RCLCPP_INFO(this->get_logger(), "=== Kt Calibration Tool ===");
    RCLCPP_INFO(this->get_logger(), "Keep wheels ELEVATED (no load)");
    RCLCPP_INFO(this->get_logger(), "Testing multiple torque values...\n");
    
    // Start calibration sequence
    calibration_timer_ = this->create_wall_timer(
      100ms, std::bind(&KtCalibrationNode::calibrationLoop, this));
  }

private:
  void feedbackCallback(const grace_msg::msg::MotorFeedback::SharedPtr msg)
  {
    latest_feedback_ = *msg;
    feedback_received_ = true;
  }
  
  void calibrationLoop()
  {
    if (calibration_stage_ == CalibrationStage::IDLE)
    {
      // Stop motors before starting
      sendTorqueCommand(0.0, 0.0);
      rclcpp::sleep_for(1s);
      calibration_stage_ = CalibrationStage::TEST_LEFT;
      test_index_ = 0;
      RCLCPP_INFO(this->get_logger(), "\n--- LEFT MOTOR CALIBRATION ---");
    }
    else if (calibration_stage_ == CalibrationStage::TEST_LEFT)
    {
      testMotor(true);  // test left
    }
    else if (calibration_stage_ == CalibrationStage::TEST_RIGHT)
    {
      testMotor(false);  // test right
    }
    else if (calibration_stage_ == CalibrationStage::DONE)
    {
      calibration_timer_->cancel();
      printResults();
      rclcpp::shutdown();
    }
  }
  
  void testMotor(bool is_left)
  {
    if (test_index_ >= test_torques_.size())
    {
      // Move to next stage
      if (is_left)
      {
        calibration_stage_ = CalibrationStage::TEST_RIGHT;
        test_index_ = 0;
        RCLCPP_INFO(this->get_logger(), "\n--- RIGHT MOTOR CALIBRATION ---");
      }
      else
      {
        calibration_stage_ = CalibrationStage::DONE;
      }
      return;
    }
    
    double torque_cmd = test_torques_[test_index_];
    
    // Phase 0: Make sure motor is stopped before starting new test
    if (sample_count_ == -1)
    {
      sendTorqueCommand(0.0, 0.0);
      RCLCPP_INFO(this->get_logger(), "Stopping motor, waiting for coast-down...");
      rclcpp::sleep_for(1500ms);  // Longer wait for complete stop
      sample_count_ = 0;
      return;
    }
    
    // Phase 1: Send command
    if (sample_count_ == 0)
    {
      if (is_left)
        sendTorqueCommand(torque_cmd, 0.0);
      else
        sendTorqueCommand(0.0, torque_cmd);
      
      RCLCPP_INFO(this->get_logger(), "Testing %s motor @ %.0f fw units...",
                  is_left ? "LEFT" : "RIGHT", torque_cmd);
      sample_count_++;
      rclcpp::sleep_for(150ms);  // Short wait to start acceleration
      return;
    }
    
    // Phase 2: Sample iq during acceleration phase
    if (sample_count_ < samples_per_test_)
    {
      if (feedback_received_)
      {
        double iq = is_left ? latest_feedback_.iq_left : latest_feedback_.iq_right;
        double rpm = is_left ? latest_feedback_.rpm_left : latest_feedback_.rpm_right;
        
        // Sample during low-to-moderate speed (acceleration phase, not max speed)
        if (std::abs(rpm) < 400.0)
        {
          iq_samples_.push_back(iq);
          RCLCPP_INFO(this->get_logger(), "  Sample %d: iq = %.3f A, rpm = %.0f", 
                      sample_count_, iq, rpm);
        }
        sample_count_++;
      }
      rclcpp::sleep_for(50ms);
      return;
    }
    
    // Phase 3: Calculate and prepare for next test
    if (!iq_samples_.empty())
    {
      double iq_avg = std::accumulate(iq_samples_.begin(), iq_samples_.end(), 0.0) 
                      / iq_samples_.size();
      
      // Kt = Km × (iq / torque_cmd)
      // Using Km = 0.15 N·m/A as typical hoverboard motor constant
      constexpr double Km = 0.15;
      double kt = Km * (std::abs(iq_avg) / torque_cmd);
      
      RCLCPP_INFO(this->get_logger(), "  iq_avg = %.3f A → Kt = %.6f N·m/fw_unit\n",
                  iq_avg, kt);
      
      if (is_left)
        kt_left_samples_.push_back(kt);
      else
        kt_right_samples_.push_back(kt);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "  No valid samples (wheel spinning too fast)\n");
    }
    
    // Reset for next test
    iq_samples_.clear();
    sample_count_ = -1;  // Will trigger stop phase
    test_index_++;
  }
  
  void sendTorqueCommand(double left, double right)
  {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {left, right};
    motor_cmd_pub_->publish(msg);
  }
  
  void printResults()
  {
    RCLCPP_INFO(this->get_logger(), "\n========================================");
    RCLCPP_INFO(this->get_logger(), "       CALIBRATION RESULTS");
    RCLCPP_INFO(this->get_logger(), "========================================\n");
    
    if (!kt_left_samples_.empty())
    {
      double kt_left_avg = std::accumulate(kt_left_samples_.begin(), 
                                           kt_left_samples_.end(), 0.0) 
                           / kt_left_samples_.size();
      RCLCPP_INFO(this->get_logger(), "LEFT MOTOR:");
      RCLCPP_INFO(this->get_logger(), "  Kt = %.6f N·m/fw_unit (avg from %zu tests)",
                  kt_left_avg, kt_left_samples_.size());
    }
    
    if (!kt_right_samples_.empty())
    {
      double kt_right_avg = std::accumulate(kt_right_samples_.begin(), 
                                            kt_right_samples_.end(), 0.0) 
                            / kt_right_samples_.size();
      RCLCPP_INFO(this->get_logger(), "RIGHT MOTOR:");
      RCLCPP_INFO(this->get_logger(), "  Kt = %.6f N·m/fw_unit (avg from %zu tests)",
                  kt_right_avg, kt_right_samples_.size());
    }
    
    if (!kt_left_samples_.empty() && !kt_right_samples_.empty())
    {
      double kt_avg = (std::accumulate(kt_left_samples_.begin(), kt_left_samples_.end(), 0.0) +
                       std::accumulate(kt_right_samples_.begin(), kt_right_samples_.end(), 0.0))
                      / (kt_left_samples_.size() + kt_right_samples_.size());
      
      RCLCPP_INFO(this->get_logger(), "\n--- RECOMMENDED VALUE ---");
      RCLCPP_INFO(this->get_logger(), "Use Kt = %.6f N·m/fw_unit", kt_avg);
      RCLCPP_INFO(this->get_logger(), "\nUpdate in:");
      RCLCPP_INFO(this->get_logger(), "  grace_controller/lqr_balancing_controller.py");
      RCLCPP_INFO(this->get_logger(), "  grace_firmware/grace_interface.hpp");
      RCLCPP_INFO(this->get_logger(), "\n========================================\n");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Calibration failed - no valid samples");
    }
  }

  enum class CalibrationStage
  {
    IDLE,
    TEST_LEFT,
    TEST_RIGHT,
    DONE
  };
  
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_cmd_pub_;
  rclcpp::Subscription<grace_msg::msg::MotorFeedback>::SharedPtr motor_feedback_sub_;
  rclcpp::TimerBase::SharedPtr calibration_timer_;
  
  grace_msg::msg::MotorFeedback latest_feedback_;
  bool feedback_received_ = false;
  
  CalibrationStage calibration_stage_ = CalibrationStage::IDLE;
  size_t test_index_ = 0;
  int sample_count_ = -1;  // Start at -1 to trigger initial stop phase
  const int samples_per_test_ = 12;
  
  std::vector<double> test_torques_ = {250.0, 200.0, 150.0, 100.0};  // High to low
  std::vector<double> iq_samples_;
  std::vector<double> kt_left_samples_;
  std::vector<double> kt_right_samples_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KtCalibrationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
