#include <memory>
#include <chrono>
#include <string>

#include "bumperbot_firmware/motor.hpp"
#include "rclcpp/rclcpp.hpp"

const std::string GPIO_CHIP = "gpiochip4";  // Raspberry Pi 5
const int MOTOR_IN1 = 27;
const int MOTOR_IN2 = 17;
const int MOTOR_ENA = 18;
const int MOTOR_ENC_A = 23;
const int MOTOR_ENC_B = 24;
const double MOTOR_KP = 12.8;
const double MOTOR_KI = 8.3;
const double MOTOR_KD = 0.1;

class SimpleMotorControl : public rclcpp::Node
{
public:
  SimpleMotorControl() : Node("simple_motor_control")
  {
    motor_ = std::make_unique<bumperbot_firmware::Motor>(
      GPIO_CHIP, MOTOR_IN1, MOTOR_IN2, MOTOR_ENA, MOTOR_ENC_A, MOTOR_ENC_B, MOTOR_KP, MOTOR_KI,
      MOTOR_KD);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      [this]() {
        motor_->setVelocity(15.0);
        double position = motor_->getPosition();
        double velocity = motor_->getVelocity();
        RCLCPP_INFO(this->get_logger(), "Position: %.2f ticks, Velocity: %.2f rad/s", position, velocity);
      }); 
  }

  ~SimpleMotorControl()
  {
    if (motor_) motor_->stop();
  }

private:
  std::unique_ptr<bumperbot_firmware::Motor> motor_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleMotorControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
