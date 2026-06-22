#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <Arduino.h>

namespace Temperature
{
    /**
     * @brief Reads inner/outer temperature sensors dynamically based on incoming buffer codes
     * and streams to the UART DMA handler.
     */
    void checkSensors(char *buff, int count);

    /**
     * @brief Averages the internal system fan pulse feedback to measure rotational speed.
     */
    void checkFans(void);
}

#endif // TEMPERATURE_H
