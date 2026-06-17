#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

#include "DueStepper.hpp"

#define MOTORS_COUNT     7
#define LIMIT_PINS_COUNT 10

namespace Motors
{
    /**
     * @brief Initializes limit switches (inputs + EXTI interrupts) and configures
     * the SAM3X Timer/Counters for all 7 hardware steppers. Synchonizes initial state to host.
     */
    void setup(void);

    /**
     * @brief Periodic safety net. Continuously verifies the state of all physical limit
     * switches in case an EMI transient caused a missed hardware interrupt edge.
     */
    void loop(void);

    // Controllers

    /**
     * @brief Parses an open-loop movement request from Serial and starts the stepper timer.
     * Movement is automatically blocked if the target limit switch is physically active.
     */
    void startMotor(char *, int);

    /**
     * @brief Parses a closed-loop movement request. The stepper TC interrupt will
     * automatically halt the motor when the associated Encoder reaches the target position.
     */
    void moveToEncoderPosition(char *, int);

    /**
     * @brief Instantly halts a specific motor and disables its TC hardware timer.
     */
    void stopMotor(uint8_t, MotorStopReason);

    /**
     * @brief Blindly transmits the current state of all limit switches to the host.
     */
    void sendAllLimitsValues(void);

    /**
     * @brief Toggles the hardware Enable pin (nEn) for a specific motor driver.
     */
    void enableMotor(char *, int);

    /**
     * @brief Software interlock: Instantly stops all motors and ignores any future
     * movement commands until unlocked.
     */
    void lockAlignment(void);

    /**
     * @brief Releases the software interlock, allowing motors to move again.
     */
    void unlockAlignment(void);
}

#endif // MOTORS_H
