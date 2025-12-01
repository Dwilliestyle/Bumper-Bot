#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <string>
#include <thread>
#include <atomic>
#include <chrono>

#include <gpiod.hpp>

namespace bumperbot_firmware
{
/**
 * @class Encoder
 * @brief Manages a quadrature encoder using libgpiod event monitoring.
 *
 * This class spawns a dedicated thread to monitor edge events on
 * two encoder pins (A and B). It decodes the quadrature signals
 * to maintain a thread-safe position count.
 */
class Encoder {
public:
    /**
     * @brief Constructs an Encoder object.
     * @param chip_name The name of the GPIO chip (e.g., "gpiochip0").
     * @param pin_a The BCM pin number for Channel A.
     * @param pin_b The BCM pin number for Channel B.
     */
    Encoder(const std::string & chip_name, int pin_a, int pin_b);

    /**
     * @brief Destructor. Stops the monitoring thread and joins it.
     */
    ~Encoder();

    /**
     * @brief Gets the current accumulated encoder position.
     * @return The thread-safe position count.
     */
    long getPosition();

    /**
     * @brief Gets the current velocity in radians per second.
     * @return The current velocity.
     */
    double getVelocity();

    /**
     * @brief Resets the encoder position to zero.
     */
    void reset();

private:
    inline static const double DECODING_FACTOR = 4.0; // 4 counts per pulse for quadrature decoding
    inline static const double PPR = 10.0; // Pulses per revolution
    inline static const double GEAR_RATIO = 35.0;
    inline static const double PERIOD = 60.0;  // seconds
    inline static const double RPM_TO_RADS = 0.10472; // Conversion factor from RPM to radians/second     

    /**
     * @brief The main function for the monitoring thread.
     */
    void monitorLoop();

    /**
     * @brief Updates the position based on the quadrature state machine.
     * @param state The new 2-bit state (A and B).
     */
    void updatePosition(int state);

    gpiod::line m_line_a;
    gpiod::line m_line_b;
    gpiod::line_bulk m_line_bulk;

    std::atomic<long> m_position;
    std::atomic<double> m_velocity;
    std::atomic<int> m_last_state;
    std::atomic<bool> m_running;
    std::chrono::time_point<std::chrono::system_clock> m_last_run;

    std::thread m_monitor_thread;
};
}  // namespace bumperbot_firmware

#endif  // ENCODER_HPP