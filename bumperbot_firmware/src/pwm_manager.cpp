#include "bumperbot_firmware/pwm_manager.hpp"

#include <unistd.h>
#include <fstream>
#include <stdexcept>
#include <iostream>

namespace bumperbot_firmware
{
PWMManager::PWMManager(const std::string & chip, int pin_ena)
: m_period_ns(0)
, m_enable(false)
, m_running(true)
{
  m_ena = std::make_unique<DigitalOut>(chip, pin_ena);
  m_pwm_thread = std::thread(&PWMManager::pwmLoop, this);
}

PWMManager::~PWMManager()
{
  enable(false);
  m_running = false;
  if(m_pwm_thread.joinable()){
    m_pwm_thread.join();
  }
}

void PWMManager::setFrequency(int freq_hz)
{
  if (freq_hz <= 0){
    std::cerr << "PWMManager::setFrequency: Invalid frequency " << freq_hz << " Hz" << std::endl;
    return;
  }
  m_period_ns = static_cast<long>(1000000000 / freq_hz);
}

void PWMManager::setDutyCycle(double percent)
{
  if (percent < 0.0){
    std::cerr << "PWMManager::setDutyCycle: Invalid duty cycle " << percent << ". Setting to 0.0." << std::endl;
    percent = 0.0;
  }
  if (percent > 1.0){
    std::cerr << "PWMManager::setDutyCycle: Invalid duty cycle " << percent << ". Setting to 1.0." << std::endl;
    percent = 1.0;
  }

  m_duty_ns = static_cast<long>(m_period_ns * percent);
}

void PWMManager::enable(bool state)
{
  m_enable = state;
}

void PWMManager::pwmLoop()
{
  while(m_running){
    if(m_enable){
      m_ena->set(true);
      usleep(m_duty_ns / 1000);
      m_ena->set(false);
      usleep((m_period_ns - m_duty_ns) / 1000);
    } else {
      m_ena->set(false);
      usleep(1000);
    }
  }
}
}  // namespace bumperbot_firmware