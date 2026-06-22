#ifndef VACUUM_H
#define VACUUM_H

#include <Arduino.h>

namespace Vacuum
{
    /**
     * @brief Initializes physical vacuum sensors, solenoid driver pins, and pneumatic
     * valves. Attaches EXTI interrupts to automatically report sensor state changes.
     */
    void setup(void);

    /**
     * @brief Software fallback loop. Polls the vacuum and air sensors at 100ms
     * intervals to guarantee protocol parity even if a hardware transient occurs.
     */
    void loop(void);

    // -------------------------------------------------------------------------
    // Setters
    // -------------------------------------------------------------------------

    /**
     * @brief Evaluates an incoming packet to adjust the PWM driving a specific Vacuum solenoid.
     */
    void setSolenoid(char *buff, int count);

    /**
     * @brief Evaluates an incoming packet to actuate the compressed air pneumatic valve.
     */
    void toggleCompressedAirValveState(char *buff, int count);

    // -------------------------------------------------------------------------
    // Getters / Upstream Triggers
    // -------------------------------------------------------------------------

    /**
     * @brief Queries the specific vacuum sensor indicated in the packet buffer.
     */
    void sendStateSensor(char *buff, int count);

    /**
     * @brief Broadcasts the current active power (PWM) of the requested solenoid line.
     */
    void getSolenoidPower(const char *buff, uint32_t count);

    /**
     * @brief Sends the explicit pneumatic state of the compressed air valve.
     */
    void sendCompressedAirValveState(void);

    /**
     * @brief Sends the measured physical state of the compressed air sensor line.
     */
    void sendCompressedAirSensorState(void);
}

#endif // VACUUM_H
