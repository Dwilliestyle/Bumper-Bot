#ifndef PID_HPP
#define PID_HPP

#include <chrono>

namespace bumperbot_firmware
{
class PID
{
public:
  /**
   * @brief Constructs a PID controller with specified gains.
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   */
  PID(double p = 0.0, double i = 0.0, double d = 0.0);

  /**
   * @brief Destructor.
   */
  ~PID() = default;

  /**
   * @brief Sets the PID gains.
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   */
  void setGains(double p, double i, double d);

  /**
   * @brief Sets the output limits.
   * @param min Minimum output value.
   * @param max Maximum output value.
   */
  void setLimits(double min, double max);

  /**
   * @brief Computes the PID output.
   * @param input The current input value.
   * @param setpoint The desired setpoint value.
   * @return The computed control output.
   */
  double compute(double input, double setpoint);

private:
  double kp_;
  double ki_;
  double kd_;
  double last_input_;
  double output_sum_;
  double min_output_;
  double max_output_;
  std::chrono::time_point<std::chrono::system_clock> last_run_;
};
}  // namespace bumperbot_firmware
#endif  // PID_HPP