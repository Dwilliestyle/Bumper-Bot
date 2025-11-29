
#include "bumperbot_firmware/digital_out.hpp"

#include <stdexcept>

namespace bumperbot_firmware
{
DigitalOut::DigitalOut(const std::string & chip_name, int pin)
{
  // Open the chip
  gpiod::chip chip(chip_name);
  if (!chip) {
    throw std::runtime_error("Could not open GPIO chip: " + chip_name);
  }

  // Get the line
  m_line = chip.get_line(pin);
  if (!m_line) {
    throw std::runtime_error("Could not get GPIO line: " + std::to_string(pin));
  }

  // Request the line as an output
  gpiod::line_request config;
  config.consumer = "motor_control";
  config.request_type = gpiod::line_request::DIRECTION_OUTPUT;

  m_line.request(config, 0);  // Default to LOW
  if (!m_line.is_requested()) {
    throw std::runtime_error("Could not request GPIO line: " + std::to_string(pin));
  }
}

DigitalOut::~DigitalOut()
{
  if (m_line && m_line.is_requested()) {
    m_line.set_value(0);  // Set low before releasing
    m_line.release();
  }
}

void DigitalOut::set(bool value)
{
  if (m_line) {
    m_line.set_value(value ? 1 : 0);
  }
}
}  // namespace bumperbot_firmware