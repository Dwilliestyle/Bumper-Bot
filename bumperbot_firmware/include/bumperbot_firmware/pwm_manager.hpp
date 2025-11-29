#ifndef PWM_MANAGER_HPP
#define PWM_MANAGER_HPP

#include <string>
#include <thread>
#include <atomic>
#include <memory>

#include "bumperbot_firmware/digital_out.hpp"

namespace bumperbot_firmware
{
/**
 * @class PWMManager
 * @brief Manages a hardware PWM channel via the Linux sysfs interface.
 *
 * This class abstracts the file I/O required to control a hardware
 * PWM pin, which libgpiod does not support.
 */
class PWMManager {
public:
    /**
     * @brief Constructs a PWMManager object.
     */
    PWMManager(const std::string & chip_name, int pin);

    /**
     * @brief Destructor. Disables and unexports the PWM channel.
     */
    ~PWMManager();

    /**
     * @brief Sets the frequency of the PWM signal.
     * @param freq_hz Frequency in Hertz.
     */
    void setFrequency(int freq_hz);

    /**
     * @brief Sets the duty cycle of the PWM signal.
     * @param percent Duty cycle as a fraction (0.0 to 1.0).
     */
    void setDutyCycle(double percent);

    /**
     * @brief Enables or disables the PWM output.
     * @param state true to enable, false to disable.
     */
    void enable(bool state);

private:
    std::unique_ptr<DigitalOut> m_ena;
    std::atomic<long> m_period_ns;
    std::atomic<long> m_duty_ns;
    std::atomic<bool> m_enable;
    std::atomic<bool> m_running;
    std::thread m_pwm_thread;

    /**
     * @brief The main loop for generating PWM signals.
     */
    void pwmLoop();
};
}  // namespace bumperbot_firmware

#endif  // PWM_MANAGER_HPP
