#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <memory>
#include <string>

#include "bumperbot_firmware/digital_out.hpp"
#include "bumperbot_firmware/encoder.hpp"
#include "bumperbot_firmware/pid.hpp"
#include "bumperbot_firmware/pwm_manager.hpp"

namespace bumperbot_firmware
{
/**
 * @class Motor
 * @brief Provides a high-level interface to control a single DC motor.
 *
 * This class composes DigitalOut (for direction) and PWMManager (for speed)
 * to provide a simple API for controlling one motor on an L298N driver.
 */
class Motor
{
public:
  /**
   * @brief Constructs a Motor object.
   * @param chip The GPIO chip name (e.g., "gpiochip0").
   * @param pin_in1 BCM pin for L298N IN1 or IN3.
   * @param pin_in2 BCM pin for L298N IN2 or IN4.
   * @param pwmchip The pwmchip number (e.g., 0).
   * @param pwm_chan The PWM channel (e.g., 0 for ENA, 1 for ENB).
   * @param pwm_freq The desired PWM frequency in Hz (default 10000).
   */
  Motor(
    const std::string & chip, int pin_in1, int pin_in2, int pin_ena, int pin_enc_a, int pin_enc_b,
    double kp, double ki, double kd);

  /**
   * @brief Destructor. Stops the motor.
   */
  ~Motor();

  /**
   * @brief Sets the motor speed.
   * @param velocity Speed in radians per second.
   */
  void setVelocity(double velocity);

  /**
   * @brief Gets the current motor speed setting.
   * @return Current speed in radians per second.
   */
  double getVelocity() const;

  /**
   * @brief Gets the current encoder position.
   * @return Current position in ticks.
   */
  double getPosition() const;

  /**
   * @brief Sets the motor to move forward.
   */
  void forward();

  /**
   * @brief Sets the motor to move in reverse.
   */
  void reverse();

  /**
   * @brief Stops the motor by disabling PWM (coasting).
   */
  void stop();

  /**
   * @brief Stops the motor by shorting the terminals (braking).
   */
  void brake();

private:
  std::unique_ptr<DigitalOut> in1_;
  std::unique_ptr<DigitalOut> in2_;
  std::unique_ptr<PWMManager> ena_;
  std::unique_ptr<Encoder> encoder_;
  std::unique_ptr<PID> pid_;
};
}  // namespace bumperbot_firmware

#endif  // MOTOR_HPP