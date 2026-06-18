#include "stops.h"
#include "SerialTXHandler.h"
#include <Arduino.h>

static uint32_t previousStopsArdkoTime = 0;

unsigned int StatesArtDeco[4] = {LOW, LOW, LOW, LOW};

unsigned int stopStatesARTDECO[4] = {
    ADFG_STOP,
    ADFD_STOP,
    ADBG_STOP,
    ADBD_STOP};

void setupArtDecoStops()
{
    pinMode(ADFG_STOP, INPUT);
    pinMode(ADFD_STOP, INPUT);
    pinMode(ADBG_STOP, INPUT);
    pinMode(ADBD_STOP, INPUT);
}

void sendAllStopARTDECO()
{
    for (int i = 0; i < 4; i++)
    {
        uint8_t buff[] = {'K', i + '1', digitalRead(stopStatesARTDECO[i])};
        Com::send(serial_packet_t(buff, sizeof(buff)));
    }
}

/*void sendStateStopARTDECO(char* buff, int count){
    if(count != 3)
    {
        return;
    }
    int index = buff[2] - '1';
    if(index < 0 || index > 3)
    {
        return;
    }
    byte buffOut[4] = {3, 'K', buff[2], digitalRead(stopStatesARTDECO[index])};
    Serial.write(buffOut, 4);
}*/

void sendStateStopARTDECO(char *buff, int count)
{
    if (count >= 3)
    {
        int index = buff[2] - '1';

        if (index < 0 || index > 3)
            return;

        uint8_t buffOut[] = {'K', buff[2], digitalRead(stopStatesARTDECO[index]) + '0'};
        Com::send(serial_packet_t(buffOut, sizeof(buffOut)));
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            uint8_t buff[] = {'K', i + '1', digitalRead(stopStatesARTDECO[i]) + '0'};
            Com::send(serial_packet_t(buff, sizeof(buff)));
        }
    }
}

void verificationStopsArdko()
{
    if (millis() - previousStopsArdkoTime >= 100)
    {
        previousStopsArdkoTime = millis();
        for (int i = 0; i < 4; ++i)
        {
            const bool state = digitalRead(stopStatesARTDECO[i]);

            if (state != StatesArtDeco[i])
            {
                uint8_t ardkoByteCode = i + '1';
                uint8_t buff[]        = {'K', ardkoByteCode, state + '0'};

                Com::send(serial_packet_t(buff, sizeof(buff)));
                StatesArtDeco[i] = state;
            }
        }
    }
}