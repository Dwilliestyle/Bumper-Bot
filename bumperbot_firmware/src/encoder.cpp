#include "bumperbot_firmware/encoder.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>

namespace bumperbot_firmware
{
Encoder::Encoder(const std::string & chip_name, int pin_a, int pin_b)
: m_position(0), m_last_state(0), m_running(false)
{
  gpiod::chip chip(chip_name);
  if (!chip) {
    throw std::runtime_error("Could not open GPIO chip: " + chip_name);
  }

  m_line_a = chip.get_line(pin_a);
  m_line_b = chip.get_line(pin_b);
  if (!m_line_a || !m_line_b) {
    throw std::runtime_error("Could not get encoder GPIO lines");
  }

  // Request both lines for "both edges" event monitoring
  gpiod::line_request config;
  config.consumer = "encoder_monitor";
  config.request_type = gpiod::line_request::EVENT_BOTH_EDGES;
  config.flags = gpiod::line_request::FLAG_BIAS_PULL_UP;

  m_line_a.request(config);
  m_line_b.request(config);
  if (!m_line_a.is_requested() || !m_line_b.is_requested()) {
    throw std::runtime_error("Could not request encoder lines");
  }

  // Add lines to a bulk object for efficient waiting
  m_line_bulk.append(m_line_a);
  m_line_bulk.append(m_line_b);

  // Read initial state
  int a = m_line_a.get_value();
  int b = m_line_b.get_value();
  m_last_state = (a << 1) | b;
  m_position = 0;

  // Start the monitoring thread
  m_running = true;
  m_monitor_thread = std::thread(&Encoder::monitorLoop, this);
}

Encoder::~Encoder()
{
  m_running = false;
  if (m_monitor_thread.joinable()) {
    m_monitor_thread.detach();
  }
}

long Encoder::getPosition() { return m_position.load(); }

double Encoder::getVelocity() { return m_velocity.load(); }

void Encoder::reset()
{
  m_position.store(0);
  m_velocity.store(0.0);
}

void Encoder::monitorLoop()
{
  // This table defines the state transitions that increment or decrement
  // the position.
  // Index: (last_state << 2) | current_state
  // Value: 0 = no change, 1 = increment, -1 = decrement
  const std::vector<int8_t> state_table = {
    0,  -1, 1,  0,
    1,  0,  0,  -1,
    -1, 0,  0,  1,
    0,  1,  -1, 0
  };

  while (m_running.load()) {
    // Block until an event occurs on either line
    auto triggered_bulk = m_line_bulk.event_wait(std::chrono::seconds(1));

    if (triggered_bulk && !triggered_bulk.empty()) {
      // Read new state
      int a = m_line_a.get_value();
      int b = m_line_b.get_value();
      int current_state = (a << 1) | b;
      int last = m_last_state.load();

      if (current_state != last) {
        int index = (last << 2) | current_state;
        m_position.store(m_position.load() + state_table[index]);
        // Update velocity
        auto now = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration<double>(now - m_last_run).count();
        if (elapsed > 0) {
          m_velocity.store((state_table[index] * PERIOD * RPM_TO_RADS) / (DECODING_FACTOR * PPR * GEAR_RATIO * elapsed));
        } else {
          m_velocity.store(0.0);
        }
        m_last_state.store(current_state);
        m_last_run = std::chrono::system_clock::now();
      }
    }
  }
}
}  // namespace bumperbot_firmware