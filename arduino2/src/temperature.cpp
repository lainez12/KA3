#include "temperature.h"

static int fanVoltages[NUMBER_IT];

static int mean(int *list, uint32_t size);

void checkTemperature(byte *buff, int count)
{
    if (count < 3)
    {
        return;
    }
    int temp(0);
    if (buff[2] == '0')
    {
        float mesure(0);
        for (int i = 0; i < 5; ++i)
        {

            mesure += (float)analogRead(SONDE_TEMP_INNER);
        }
        mesure /= 5.0;
        temp = (int)mesure;
    }
    else if (buff[2] == '1')
    {
        float calculMoyenne(0), mesure(0);
        for (int i = 0; i < 5; ++i)
        {

            mesure += (float)analogRead(SONDE_TEMP_OUTER);
        }
        mesure /= 5.0;
        temp = (int)mesure;
    }
    if (temp != -1)
    {
        byte buffTemp[6] = {5, 'I', 'T', buff[2], highByte(temp), lowByte(temp)};
        Serial.write(buffTemp, 6);
    }
}

void checkFans()
{
    static bool enoughValuesGathered = false;
    static uint8_t currentValueIdx   = 0;

    fanVoltages[currentValueIdx] = analogRead(FAN_TURNS);
    currentValueIdx              = (currentValueIdx + 1) % NUMBER_IT;

    if (!enoughValuesGathered)
    {
        if (currentValueIdx == 0) // Value equals 0 here only when one full cycle of value gathering has finished
            return;
        enoughValuesGathered = true;
    }

    int fanVal   = mean(fanVoltages, NUMBER_IT);
    byte buff[5] = {4, 'I', 'F', highByte(fanVal), lowByte(fanVal)};

    Serial.write(buff, 5);
}

static int mean(int *list, uint32_t size)
{
    long sumValue = 0;

    for (uint32_t k = 0; k < size; ++k)
        sumValue += list[k];
    return sumValue / size;
}

void clearArrayMoyennageFanVoltage()
{
    /*for (unsigned int i = 0; i < LedVoltage.size() ; ++i)
    {
      LedVoltage[i].clear();
    }

    for (unsigned int i = 0; i < LedCurrent.size() ; ++i)
    {
      LedCurrent[i].clear();
    }*/
    // fanVoltages.clear();
}

void clearArrayMoyennageTempVoltage()
{
    /*for (unsigned int i = 0; i < LedVoltage.size() ; ++i)
    {
      LedVoltage[i].clear();
    }

    for (unsigned int i = 0; i < LedCurrent.size() ; ++i)
    {
      LedCurrent[i].clear();
    }*/
    // TempVoltage.clear();
}
