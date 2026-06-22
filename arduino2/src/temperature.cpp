#include "temperature.h"
#include "SerialTXHandler.h"
#include "pins.h"

#define NUMBER_IT 5

namespace Temperature
{
    namespace
    {
        int fanVoltages[NUMBER_IT];

        int _calculateMean(int *list, uint32_t size)
        {
            long sumValue = 0;
            for (uint32_t k = 0; k < size; ++k)
                sumValue += list[k];
            return sumValue / size;
        }
    }

    void checkSensors(char *buff, int count)
    {
        if (count < 3)
            return;

        int temp     = -1;
        float mesure = 0.0f;

        if (buff[2] == '0')
        {
            for (int i = 0; i < 5; ++i)
                mesure += (float)analogRead(SONDE_TEMP_INNER);

            temp = (int)(mesure / 5.0f);
        }
        else if (buff[2] == '1')
        {
            for (int i = 0; i < 5; ++i)
                mesure += (float)analogRead(SONDE_TEMP_OUTER);

            temp = (int)(mesure / 5.0f);
        }

        if (temp != -1)
        {
            const uint8_t buffTemp[] = {'I', 'T', (uint8_t)buff[2], highByte(temp), lowByte(temp)};
            Com::send(serial_packet_t(buffTemp, sizeof(buffTemp)));
        }
    }

    void checkFans(void)
    {
        static bool enoughValuesGathered = false;
        static uint8_t currentValueIdx   = 0;

        fanVoltages[currentValueIdx] = analogRead(FAN_TURNS);
        currentValueIdx              = (currentValueIdx + 1) % NUMBER_IT;

        if (!enoughValuesGathered)
        {
            if (currentValueIdx == 0)
                enoughValuesGathered = true;
            else
                return;
        }

        const int fanVal     = _calculateMean(fanVoltages, NUMBER_IT);
        const uint8_t buff[] = {'I', 'F', highByte(fanVal), lowByte(fanVal)};

        Com::send(serial_packet_t(buff, sizeof(buff)));
    }
}
