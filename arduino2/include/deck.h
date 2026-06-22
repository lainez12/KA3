#ifndef DECK_H
#define DECK_H

#include <Arduino.h>

namespace Deck
{
    /**
     * @brief Initializes the Deck motor pins, Timer-based PWM hardware, and attaches
     * bare-metal EXTI hardware interrupts for the alignment and insolation limit switches.
     */
    void setup(void);

    /**
     * @brief Periodic background loop. Monitors the analog torque feedback to enforce
     * the programmed torque limit, and acts as a 100ms safety net against missed limit switch edges.
     */
    void loop(void);

    // -------------------------------------------------------------------------
    // Controllers (Called from Serial Parsing)
    // -------------------------------------------------------------------------

    /**
     * @brief Parses a continuous movement request and starts the motor. Movement is
     * automatically blocked if the target limit switch is physically active.
     *
     * @param buff Pointer to the raw serial payload.
     * @param count Total byte length of the payload.
     */
    void startMotor(char *buff, int count);

    /**
     * @brief Updates the maximum allowed analog torque limit for a specific direction.
     *
     * @param buff Pointer to the raw serial payload.
     * @param count Total byte length of the payload.
     */
    void setTorqueLimit(char *buff, int count);

    /**
     * @brief Instantly halts the deck motor and disables its PWM generation.
     */
    void stopMotor(void);

    /**
     * @brief Blindly transmits the current state of both deck limit switches to the host.
     */
    void sendAllLimitsValues(void);
}

#endif // DECK_H
