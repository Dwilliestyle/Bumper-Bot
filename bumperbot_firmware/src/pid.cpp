#include "bumperbot_firmware/pid.hpp"

namespace bumperbot_firmware
{
PID::PID(double p, double i, double d) : kp_(p), ki_(i), kd_(d)
{}

void PID::setGains(double p, double i, double d)
{
    kp_ = p;
    ki_ = i;
    kd_ = d;
}

double PID::compute(double error, double dt)
{
    if (dt <= 0.0){
        return 0.0;
    }

    double error_dot = d_error_;

    // Calculate the derivative error
    if (dt > 0.0)
    {
        error_dot = (error - p_error_last_) / dt;
        p_error_last_ = error;
    }

    double p_term, d_term, i_term;
    p_error_ = error;
    d_error_ = error_dot;

    p_term = kp_ * p_error_;
    i_error_ += dt * p_error_;
    i_term = ki_ * i_error_;
    d_term = kd_ * d_error_;

    // Compute the command
    cmd_ = p_term + i_term + d_term;
    return cmd_;
}
}  // namespace bumperbot_firmware