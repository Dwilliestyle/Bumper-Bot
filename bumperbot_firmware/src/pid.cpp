#include "bumperbot_firmware/pid.hpp"

namespace bumperbot_firmware
{
PID::PID(double p, double i, double d)
: kp_(p), ki_(i), kd_(d), min_output_(0.0), max_output_(1.0), last_input_(0.0), output_sum_(0.0)
{
  last_run_ = std::chrono::system_clock::now();
}

void PID::setGains(double p, double i, double d)
{
  kp_ = p;
  ki_ = i;
  kd_ = d;
}

void PID::setLimits(double min, double max)
{
  min_output_ = min;
  max_output_ = max;
}

double PID::compute(double input, double setpoint)
{
  auto dt = std::chrono::duration<double>(std::chrono::system_clock::now() - last_run_).count();
  double error = setpoint - input;
  double dinput = input - last_input_;
  output_sum_ += (ki_ * error);

  if (output_sum_ > max_output_)
    output_sum_ = max_output_;
  else if (output_sum_ < min_output_)
    output_sum_ = min_output_;

  double output = kp_ * error + output_sum_ - kd_ * dinput;
  if (output > max_output_) {
    output_sum_ -= output - max_output_;
    output = max_output_;
  } else if (output < min_output_) {
    output_sum_ += min_output_ - output;
    output = min_output_;
  }

  last_run_ = std::chrono::system_clock::now();
  last_input_ = input;
  return output;
}
}  // namespace bumperbot_firmware