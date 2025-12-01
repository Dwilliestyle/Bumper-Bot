#include "bumperbot_firmware/motor.hpp"

#include <stdexcept>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

namespace bumperbot_firmware
{
Motor::Motor(
  const std::string & chip, int pin_in1, int pin_in2, int pin_ena, int pin_enc_a, int pin_enc_b,
  double kp, double ki, double kd)
{
  in1_ = std::make_unique<DigitalOut>(chip, pin_in1);
  in2_ = std::make_unique<DigitalOut>(chip, pin_in2);
  ena_ = std::make_unique<PWMManager>(chip, pin_ena);
  encoder_ = std::make_unique<Encoder>(chip, pin_enc_a, pin_enc_b);
  pid_ = std::make_unique<PID>(kp, ki, kd);
  pid_->setLimits(0.0, 1.0);

  ena_->setFrequency(1000);
  ena_->setDutyCycle(0.0);
  ena_->enable(true);
}

Motor::~Motor()
{
  stop();
}

void Motor::setVelocity(double velocity)
{
  if(velocity > 0) {
    forward();
  } else if (velocity < 0) {
    reverse();
  } else {
    stop();
    return;
  }
  double control_signal = pid_->compute(std::abs(encoder_->getVelocity()), std::abs(velocity));
  ena_->setDutyCycle(control_signal);
}

double Motor::getVelocity() const
{
  return encoder_->getVelocity();
}

double Motor::getPosition() const
{
  return encoder_->getPosition();
}

void Motor::forward()
{
  in1_->set(true);
  in2_->set(false);
  ena_->enable(true);
}

void Motor::reverse()
{
  in1_->set(false);
  in2_->set(true);
  ena_->enable(true);
}

void Motor::stop()
{
  ena_->enable(false);
  ena_->setDutyCycle(0.0);
  in1_->set(false);
  in2_->set(false);
}
}  // namespace bumperbot_firmware