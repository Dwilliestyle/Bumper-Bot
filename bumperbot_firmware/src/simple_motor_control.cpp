#include <memory>

#include "bumperbot_firmware/motor.hpp"
#include "rclcpp/rclcpp.hpp"

const std::string GPIO_CHIP = "gpiochip4";  // Raspberry Pi 5
const int MOTOR_IN1 = 17;
const int MOTOR_IN2 = 27;
const int MOTOR_ENA = 18;
const int MOTOR_ENC_A = 24;
const int MOTOR_ENC_B = 25;

class SimpleMotorControl : public rclcpp::Node
{
public:
  SimpleMotorControl() : Node("simple_motor_control")
  {
    motor_ = std::make_unique<bumperbot_firmware::Motor>(GPIO_CHIP, MOTOR_IN1, MOTOR_IN2, MOTOR_ENA, MOTOR_ENC_A, MOTOR_ENC_B);

    // Example: Set speed to 50% and move forward
    motor_->setVelocity(0.5);
    motor_->forward();
  }

  ~SimpleMotorControl()
  {
    if (motor_) motor_->stop();
  }

private:
  std::unique_ptr<bumperbot_firmware::Motor> motor_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleMotorControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
