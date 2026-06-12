#include <Arduino.h>

#include "DueTimer.hpp"
#include "EMAFilter.hpp"
#include "SerialTXHandler.h"
#include "definitions.h"
#include "forceSensors.h"
#include "pins.h"
#include "timerPriorities.h"

#define TOTAL_FORCE_SENSORS         3
#define FORCE_MEASUREMENT_FREQUENCY 500 // Hz

// Private scope
namespace
{
    // Pins array
    const int PIEZO[TOTAL_FORCE_SENSORS] = {CALG, CALD, CALA};
    // State variables
    uint32_t lastMeasureTimestamp                         = 0;
    volatile bool forceSensorEnabled[TOTAL_FORCE_SENSORS] = {false, false, false};
    DueTimer forceSensorMeasureTimer(FORCE_SENSORS_MEASURE_TIMER_IDX);

    uint32_t getADCValue(uint8_t sensorIdx)
    {
        return analogRead(PIEZO[sensorIdx]);
    }

    EMAFilter forceFilters[TOTAL_FORCE_SENSORS] = {
        EMAFilter([]() { return getADCValue(0); }, 200),
        EMAFilter([]() { return getADCValue(1); }, 200),
        EMAFilter([]() { return getADCValue(2); }, 200)};

    // --- Helpers ---

    bool isAnySensorEnabled()
    {
        for (uint8_t i = 0; i < TOTAL_FORCE_SENSORS; ++i)
        {
            if (forceSensorEnabled[i])
                return true;
        }
        return false;
    }

    void enableTimerIfNeeded()
    {
        if (isAnySensorEnabled())
        {
            for (uint8_t idx = 0; idx < TOTAL_FORCE_SENSORS; ++idx)
                forceFilters[idx].reset();
            forceSensorMeasureTimer.start();
        }
        else
        {
            forceSensorMeasureTimer.stop();
        }
    }

    void measureAll(void *)
    {
        for (uint8_t idx = 0; idx < TOTAL_FORCE_SENSORS; ++idx)
        {
            if (forceSensorEnabled[idx])
                forceFilters[idx].measure();
        }
    }

    void sendSensorData(uint8_t sensorIdx, uint32_t measuredForce)
    {
        char buffer[12];
        int payloadLen = snprintf(buffer, sizeof(buffer), "F%u%04lu", sensorIdx + 1, measuredForce);
        Com::send(serial_packet_t((uint8_t *)buffer, payloadLen));
    }

    void sendAll()
    {
        for (uint8_t idx = 0; idx < TOTAL_FORCE_SENSORS; ++idx)
        {
            if (forceSensorEnabled[idx])
                sendSensorData(idx, forceFilters[idx].getCurrentValue());
        }
    }
}

// Public functions
namespace ForceSensors
{
    // Force sensors setup function
    void setup(void)
    {
        pinMode(CALG, INPUT);
        pinMode(CALD, INPUT);
        pinMode(CALA, INPUT);

        forceSensorMeasureTimer.begin(FORCE_MEASUREMENT_FREQUENCY, &measureAll, nullptr);

        enableTimerIfNeeded();
    }

    // Force sensors function to be called in the main execution loop
    void loop(void)
    {
        const uint32_t currentTimeMs = millis();

        // Send sensors measured value every 100ms when any sensor is enabled
        if ((forceSensorEnabled[0] || forceSensorEnabled[1] || forceSensorEnabled[2]) &&
            (currentTimeMs - lastMeasureTimestamp) > 100)
        {
            sendAll();
            lastMeasureTimestamp = currentTimeMs;
        }
    }

    // Controllers

    void setEnabledState(char *buff, int count)
    {
        if (count != 3)
            return;

        uint8_t idx = buff[1] - '0';

        if (idx < 1 || idx > TOTAL_FORCE_SENSORS)
            return;

        const bool enabled = (buff[2] == '1');

        forceSensorEnabled[idx - 1] = enabled;
        enableTimerIfNeeded();

        const uint8_t buffer[] = {'?', 'F', (uint8_t)(idx + '0'), (uint8_t)(enabled ? '1' : '0')};
        Com::send(serial_packet_t(buffer, sizeof(buffer))); // Acknowledge to software
    }

    void sendEnabledState(char *buff, int count)
    {
        if (count != 3)
            return;

        const uint8_t idx = buff[2] - '0';

        if (idx < 1 || idx > TOTAL_FORCE_SENSORS)
            return;

        const uint8_t buffer[] = {'F', '?', (uint8_t)(idx + '0'), (uint8_t)(forceSensorEnabled[idx - 1] ? '1' : '0')};
        Com::send(serial_packet_t(buffer, sizeof(buffer)));
    }
}
