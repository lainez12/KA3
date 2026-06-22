#ifndef ARDKO_H
#define ARDKO_H

#include <Arduino.h>

namespace Ardko
{
    /**
     * @brief Initializes the 4 ArtDeco (Ardko) Mask Conveyor Limit sensors and configures
     * hardware EXTI interrupts for immediate state synchronization.
     */
    void setup(void);

    /**
     * @brief Periodic safety loop acting as a software fallback to verify limit
     * states against potential EMI transient errors.
     */
    void loop(void);

    /**
     * @brief Blindly transmits the current state of all 4 Ardko limits to the host.
     */
    void sendAllLimitsValues(void);

    /**
     * @brief Parses a serial query and sends the state of a specific Ardko limit sensor.
     *
     * @param buff Pointer to the raw serial payload.
     * @param count Total byte length of the payload.
     */
    void sendLimitValue(char *buff, int count);
}

#endif // ARDKO_H
