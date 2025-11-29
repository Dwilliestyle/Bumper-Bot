#ifndef PID_HPP
#define PID_HPP

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
     * @brief Computes the PID output.
     * @param error The current error value.
     * @param dt Time duration since the last computation.
     * @return The computed control output.
     */
    double compute(double error, double dt);

private:
    double kp_;
    double ki_;
    double kd_;
    double p_error_last_;
    double p_error_;
    double i_error_;
    double d_error_;
    double cmd_;
};
}  // namespace bumperbot_firmware
#endif  // PID_HPP