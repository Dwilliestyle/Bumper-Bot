#include "bumperbot_firmware/bumperbot_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace bumperbot_firmware
{
BumperbotInterface::BumperbotInterface() {}

BumperbotInterface::~BumperbotInterface()
{
  if (left_motor_) left_motor_->stop();
  if (right_motor_) right_motor_->stop();
}

CallbackReturn BumperbotInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  std::string gpio_chip;
  try {
    gpio_chip = info_.hardware_parameters.at("gpio_chip");
    RCLCPP_INFO(
      rclcpp::get_logger("BumperbotInterface"),
      "BumperbotInterface: Using GPIO chip: %s", gpio_chip.c_str());
  } catch (const std::out_of_range & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BumperbotInterface"),
      "BumperbotInterface: Missing 'gpio_chip' parameter in hardware_parameters");
    return CallbackReturn::ERROR;
  }

  left_motor_ = std::make_unique<bumperbot_firmware::Motor>(
    gpio_chip, LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_ENA, LEFT_MOTOR_ENC_A, LEFT_MOTOR_ENC_B,
    LEFT_MOTOR_KP, LEFT_MOTOR_KI, LEFT_MOTOR_KD);
  right_motor_ = std::make_unique<bumperbot_firmware::Motor>(
    gpio_chip, RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_ENA, RIGHT_MOTOR_ENC_A,
    RIGHT_MOTOR_ENC_B, RIGHT_MOTOR_KP, RIGHT_MOTOR_KI, RIGHT_MOTOR_KD);

  velocity_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  velocity_states_.reserve(info_.joints.size());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BumperbotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BumperbotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a velocity Interafce
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn BumperbotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Starting robot hardware ...");

  // Reset commands and states
  velocity_commands_ = {0.0, 0.0};
  position_states_ = {0.0, 0.0};
  velocity_states_ = {0.0, 0.0};

  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}

CallbackReturn BumperbotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Stopping robot hardware ...");
  if (left_motor_) left_motor_->stop();
  if (right_motor_) right_motor_->stop();

  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type BumperbotInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  try {
    velocity_states_.at(0) = left_motor_->getVelocity();
    velocity_states_.at(1) = right_motor_->getVelocity();
    position_states_.at(0) = left_motor_->getPosition();
    position_states_.at(1) = right_motor_->getPosition();
  } catch (...) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BumperbotInterface"), "BumperbotInterface: Error reading from motors");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BumperbotInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  left_motor_->setVelocity(velocity_commands_.at(0));
  right_motor_->setVelocity(velocity_commands_.at(1));
  return hardware_interface::return_type::OK;
}
}  // namespace bumperbot_firmware

PLUGINLIB_EXPORT_CLASS(bumperbot_firmware::BumperbotInterface, hardware_interface::SystemInterface)