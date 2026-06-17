#ifndef DUE_DC_MOTOR_H
#define DUE_DC_MOTOR_H

#include <Arduino.h>
#include "DuePWM.hpp"


typedef struct due_dc_motor_pins_s
{
    uint32_t disable;   // Pin to enable/disable (e.g., DECK_DISABLE 10)
    uint32_t direction; // Pin for direction (e.g., DECK_DIRECTION 11)
    uint32_t pwm;       // Pin for speed PWM (e.g., DECK_CLOCK 12)
    uint32_t torque;    // Analog pin for torque feedback (e.g., DECK_COUPLE A5)
} due_dc_motor_pins_t;

class DueDCMotor
{
public:
    DueDCMotor(due_dc_motor_pins_t pins);
    ~DueDCMotor() = default;

    // Initializes the pins and PWM hardware.
    // frequency: Frequency in Hz (20000Hz is standard to avoid hearing a buzz)
    // maxResolution: The maximum value for setSpeed() (e.g., 1000 = 100%)
    bool setup(uint32_t frequency = 20000, uint32_t maxResolution = 1000);

    // State control
    void enable(bool en); // Enables or disables the driver
    void stop(void);      // Stops the motor (speed to 0 and disables)

    // Movement control
    void setDirection(uint8_t dir);
    void setSpeed(uint32_t speed); // Sets the duty cycle (0 to maxResolution)

    // State getters
    uint8_t direction(void) const;
    bool isEnabled(void) const;
    uint32_t currentSpeed(void) const;
    
    // Analog getter
    uint32_t readTorque(void) const; // Reads the torque sensor value

private:
    DuePWM m_pwm; // Instance of the hardware PWM controller
    const due_dc_motor_pins_t m_pins;

    // Internal state
    uint8_t m_direction;
    uint32_t m_currentSpeed;
    bool m_enabled;
    uint32_t m_maxResolution;
};

#endif // DUE_DC_MOTOR_H