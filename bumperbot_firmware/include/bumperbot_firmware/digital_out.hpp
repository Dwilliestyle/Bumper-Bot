#ifndef DIGITAL_OUT_HPP
#define DIGITAL_OUT_HPP

#include <string>

#include <gpiod.hpp>


namespace bumperbot_firmware
{
/**
 * @class DigitalOut
 * @brief Manages a single GPIO output pin using libgpiod.
 *
 * This class handles requesting a GPIO line as an output and
 * provides a simple interface to set its value (HIGH/LOW).
 * The line is automatically released on destruction.
 */
class DigitalOut {
public:
    /**
     * @brief Constructs a DigitalOut object.
     * @param chip_name The name of the GPIO chip (e.g., "gpiochip0").
     * @param pin The BCM pin number.
     */
    DigitalOut(const std::string & chip_name, int pin);

    /**
     * @brief Destructor. Releases the GPIO line.
     */
    ~DigitalOut();

    /**
     * @brief Sets the state of the output pin.
     * @param value true for HIGH (1), false for LOW (0).
     */
    void set(bool value);

private:
    gpiod::line m_line;
};
}  // namespace bumperbot_firmware

#endif  // DIGITAL_OUT_HPP