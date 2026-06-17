#ifndef SENSOR_H
#define SENSOR_H

namespace ForceSensors
{
    /**
     * @brief Initializes ADC pins and the SAM3X TC (Timer/Counter) used for
     * strict 500Hz deterministic sampling.
     */
    void setup(void);

    /**
     * @brief Periodic state machine tick. Flushes the current Exponential Moving Average
     * (EMA) filtered values to the UART queue at 10Hz if any sensor is active.
     */
    void loop(void);

    // -------------------------------------------------------------------------
    // Controllers
    // -------------------------------------------------------------------------

    /**
     * @brief Parses a serial request to respond with the current ON/OFF enabled
     * state of a specific force sensor.
     *
     * @param buff Pointer to the raw serial payload.
     * @param count Total byte length of the payload.
     */
    void sendEnabledState(char *buff, int count);

    /**
     * @brief Enables or disables the hardware sampling of a specific sensor.
     * If all sensors are disabled, the hardware TC timer is powered down to save CPU.
     *
     * @param buff Pointer to the raw serial payload.
     * @param count Total byte length of the payload.
     */
    void setEnabledState(char *buff, int count);
}

#endif // FORCE_SENSORS_H