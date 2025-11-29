#include "bumperbot_firmware/motor.hpp"

#include <stdexcept>

namespace bumperbot_firmware
{
Motor::Motor(
  const std::string & chip, int pin_in1, int pin_in2, int pin_ena, int pin_enc_a, int pin_enc_b)
: m_current_speed(0.0)
{
  try {
    m_in1 = std::make_unique<DigitalOut>(chip, pin_in1);
    m_in2 = std::make_unique<DigitalOut>(chip, pin_in2);
    m_ena = std::make_unique<PWMManager>(chip, pin_ena);
    m_encoder = std::make_unique<Encoder>(chip, pin_enc_a, pin_enc_b);

    m_ena->setFrequency(1000);
    m_ena->setDutyCycle(0.0);
    m_ena->enable(true);
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to initialize motor: " + std::string(e.what()));
  }
}

Motor::~Motor()
{
  try {
    stop();
  } catch (...) {
  }
}

void Motor::setVelocity(double percent)
{
  if (percent < 0.0) percent = 0.0;
  if (percent > 1.0) percent = 1.0;
  m_current_speed = percent;
  m_ena->setDutyCycle(m_current_speed);
}

double Motor::getVelocity() const
{
  return static_cast<double>(m_encoder->getVelocity());
}

double Motor::getPosition() const
{
  return static_cast<double>(m_encoder->getPosition());
}

void Motor::forward()
{
  m_in1->set(true);
  m_in2->set(false);
  m_ena->enable(true);
  m_ena->setDutyCycle(m_current_speed);
}

void Motor::reverse()
{
  m_in1->set(false);
  m_in2->set(true);
  m_ena->enable(true);
  m_ena->setDutyCycle(m_current_speed);
}

void Motor::stop()
{
  // Coast stop
  m_ena->enable(false);
  m_ena->setDutyCycle(0.0);
  m_in1->set(false);
  m_in2->set(false);
}
}  // namespace bumperbot_firmware