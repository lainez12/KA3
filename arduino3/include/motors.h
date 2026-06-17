#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

#include "DueStepper.hpp"
#include "encoder.h"

#define MOTORS_COUNT          5
#define STANDARD_LIMITS_COUNT 13
#define ZONE_LIMITS_COUNT     3
#define LIMIT_PINS_COUNT      (STANDARD_LIMITS_COUNT + ZONE_LIMITS_COUNT)

namespace Motors
{
    /**
     * @brief Initializes limit switches (inputs + EXTI interrupts) and configures
     * the SAM3X Timer/Counters for all hardware steppers. Synchonizes initial state to host.
     */
    void setup(void);

    /**
     * @brief Periodic safety net. Continuously verifies the state of all physical limit
     * switches in case an EMI transient caused a missed hardware interrupt edge.
     */
    void loop(void);

    // -------------------------------------------------------------------------
    // Controllers
    // -------------------------------------------------------------------------

    /**
     * @brief Parses an open-loop movement request from Serial and starts the stepper timer.
     * Movement is automatically blocked if the target limit switch is physically active.
     *
     * @param buff Pointer to the raw serial payload.
     * @param count Total byte length of the payload.
     */
    void startMotor(char *buff, int count);

    /**
     * @brief Parses a closed-loop movement request. The stepper TC interrupt will
     * automatically halt the motor when the associated Encoder reaches the target position.
     *
     * @param buff Pointer to the raw serial payload.
     * @param count Total byte length of the payload.
     */
    void moveToEncoderPosition(char *buff, int count);

    /**
     * @brief Parses a software halt request from Serial. Converts the ASCII target
     * to a MotorId and issues a stop command.
     *
     * @param buff Pointer to the raw serial payload.
     * @param count Total byte length of the payload.
     */
    void stopMotorCommand(char *buff, int count);

    /**
     * @brief Instantly halts a specific motor and disables its TC hardware timer.
     *
     * @param index The MotorId of the motor to stop.
     * @param reason The reason (Limit, Software, Target reached) to broadcast back to the host.
     */
    void stopMotor(MotorId index, MotorStopReason reason);

    // -------------------------------------------------------------------------
    // Status Queries
    // -------------------------------------------------------------------------

    /**
     * @brief Forces an immediate read of all standard (motor collision) limit switches
     * and flushes any changed states to the UART DMA queue.
     */
    void checkAllStdLimitPins(void);

    /**
     * @brief Forces an immediate read of all Z limits (informational for stowage/masking zone)
     * and flushes any changed states to the UART DMA queue.
     */
    void checkAllZLimitPins(void);

    /**
     * @brief Blindly transmits the current state of all standard limit switches to the host.
     */
    void sendAllStdLimitsStatus(void);

    /**
     * @brief Blindly transmits the current state of all Z limits to the host.
     */
    void sendAllZLimitsStatus(void);
}

#endif // MOTORS_H
