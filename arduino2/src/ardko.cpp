#include "ardko.h"
#include "SerialTXHandler.h"
#include "pins.h"

// -----------------------------------------------------------------------------
// Constants & Mappings
// -----------------------------------------------------------------------------

#define ARDKO_LIMITS_COUNT 4

namespace Ardko
{
    namespace
    {
        const uint8_t limitPins[ARDKO_LIMITS_COUNT] = {
            ARDKO_FRONT_LEFT_LIMIT,
            ARDKO_FRONT_RIGHT_LIMIT,
            ARDKO_BACK_LEFT_LIMIT,
            ARDKO_BACK_RIGHT_LIMIT};

        // Cache state array
        volatile bool limitStates[ARDKO_LIMITS_COUNT] = {false, false, false, false};

        /**
         * @brief Syncs physical pin state to cache, sending notification on change.
         */
        void _syncLimitPinState(uint8_t index)
        {
            const bool state = digitalRead(limitPins[index]);

            if (state != limitStates[index])
            {
                limitStates[index]    = state;
                uint8_t ardkoByteCode = index + '1';
                uint8_t buff[]        = {'K', ardkoByteCode, (uint8_t)(state + '0')};

                Com::send(serial_packet_t(buff, sizeof(buff)));
            }
        }

        // EXTI Hardware Handlers
        void isrLimit0()
        {
            _syncLimitPinState(0);
        }
        void isrLimit1()
        {
            _syncLimitPinState(1);
        }
        void isrLimit2()
        {
            _syncLimitPinState(2);
        }
        void isrLimit3()
        {
            _syncLimitPinState(3);
        }
    }

    void setup(void)
    {
        void (*isrArray[ARDKO_LIMITS_COUNT])() = {
            isrLimit0, isrLimit1, isrLimit2, isrLimit3};

        for (uint8_t i = 0; i < ARDKO_LIMITS_COUNT; ++i)
        {
            pinMode(limitPins[i], INPUT);
            limitStates[i] = digitalRead(limitPins[i]); // Force initial sync

            attachInterrupt(digitalPinToInterrupt(limitPins[i]), isrArray[i], CHANGE);
        }
    }

    void loop(void)
    {
        static uint32_t previousLimitsTime = 0;

        // Software Fallback (Safety Net) 100ms
        if (millis() - previousLimitsTime >= 100)
        {
            previousLimitsTime = millis();
            for (uint8_t i = 0; i < ARDKO_LIMITS_COUNT; ++i)
                _syncLimitPinState(i);
        }
    }

    void sendAllLimitsValues(void)
    {
        for (uint8_t i = 0; i < ARDKO_LIMITS_COUNT; ++i)
        {
            uint8_t buff[] = {'K', (uint8_t)(i + '1'), (uint8_t)(limitStates[i] + '0')};
            Com::send(serial_packet_t(buff, sizeof(buff)));
        }
    }

    void sendLimitValue(char *buff, int count)
    {
        if (count >= 3)
        {
            int index = buff[2] - '1';

            if (index < 0 || index >= ARDKO_LIMITS_COUNT)
                return;

            uint8_t buffOut[] = {'K', (uint8_t)buff[2], (uint8_t)(limitStates[index] + '0')};
            Com::send(serial_packet_t(buffOut, sizeof(buffOut)));
        }
        else
        {
            sendAllLimitsValues();
        }
    }
}
