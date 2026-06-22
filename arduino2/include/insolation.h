#ifndef INSOLATION_H
#define INSOLATION_H

#include <Arduino.h>

namespace Insolation
{
    /**
     * @brief Configures SPI mappings and initial logic states for the Insolation LED array
     * and precision timing drivers.
     */
    void setup(void);

    /**
     * @brief Master state machine for cycle execution. Toggles UV LED outputs and controls
     * the multiplexed sensor feedback for host parity.
     */
    void loop(void);

    /**
     * @brief Translates an incoming serial packet to extract pulse durations, delays, and
     * powers. Configures the state machine to run the active phase.
     */
    void startCycle(char *buff, int count);

    /**
     * @brief Instantly forces a shutdown of the current exposure sequence, resetting SPI
     * registers and notifying the host of the halt.
     */
    void stopCycle(char code);

    /**
     * @brief Hardware interlock exposure drop. Exists for legacy compatibility with
     * external host system Emergency logic.
     */
    void interruptExposure(void);

    /**
     * @brief Retrieves the active period (ms) of the cycle currently operating.
     */
    unsigned long getCycleTime(void);

    /**
     * @brief Retrieves the amount of phases remaining in the cycle block.
     */
    int getNumberCycles(void);
}

#endif // INSOLATION_H
