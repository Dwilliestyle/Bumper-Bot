#ifndef BUMPERBOT_INTERFACE_HPP
#define BUMPERBOT_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <bumperbot_firmware/motor.hpp>

#include <vector>
#include <string>


namespace bumperbot_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class BumperbotInterface : public hardware_interface::SystemInterface
{
public:
  BumperbotInterface();
  virtual ~BumperbotInterface();

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
  static const int LEFT_MOTOR_IN1 = 17;
  static const int LEFT_MOTOR_IN2 = 27;
  static const int LEFT_MOTOR_ENA = 18;
  static const int LEFT_MOTOR_ENC_A = 23;
  static const int LEFT_MOTOR_ENC_B = 24;

  static const int RIGHT_MOTOR_IN1 = 5;
  static const int RIGHT_MOTOR_IN2 = 6;
  static const int RIGHT_MOTOR_ENA = 12;
  static const int RIGHT_MOTOR_ENC_A = 25;
  static const int RIGHT_MOTOR_ENC_B = 66;

  std::unique_ptr<bumperbot_firmware::Motor> left_motor_;
  std::unique_ptr<bumperbot_firmware::Motor> right_motor_;
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  rclcpp::Time last_run_;
};
}  // namespace bumperbot_firmware


#endif  // BUMPERBOT_INTERFACE_HPP