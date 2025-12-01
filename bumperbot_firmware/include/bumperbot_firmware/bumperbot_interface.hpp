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
  inline static const int LEFT_MOTOR_IN1 = 27;
  inline static const int LEFT_MOTOR_IN2 = 17;
  inline static const int LEFT_MOTOR_ENA = 18;
  inline static const int LEFT_MOTOR_ENC_A = 23;
  inline static const int LEFT_MOTOR_ENC_B = 24;
  inline static const double LEFT_MOTOR_KP = 12.8;
  inline static const double LEFT_MOTOR_KI = 8.3;
  inline static const double LEFT_MOTOR_KD = 0.1;

  inline static const int RIGHT_MOTOR_IN1 = 5;
  inline static const int RIGHT_MOTOR_IN2 = 6;
  inline static const int RIGHT_MOTOR_ENA = 12;
  inline static const int RIGHT_MOTOR_ENC_A = 25;
  inline static const int RIGHT_MOTOR_ENC_B = 26;
  inline static const double RIGHT_MOTOR_KP = 11.5;
  inline static const double RIGHT_MOTOR_KI = 7.5;
  inline static const double RIGHT_MOTOR_KD = 0.1;

  std::unique_ptr<bumperbot_firmware::Motor> left_motor_;
  std::unique_ptr<bumperbot_firmware::Motor> right_motor_;
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
};
}  // namespace bumperbot_firmware


#endif  // BUMPERBOT_INTERFACE_HPP