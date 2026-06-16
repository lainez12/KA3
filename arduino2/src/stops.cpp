#include "SerialTXHandler.h"
#include "stops.h"
#include <Arduino.h>

static unsigned long previousStopsArdkoTime = 0;

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
        byte buff[4] = {3, 'K', i + '1', digitalRead(stopStatesARTDECO[i])};
        Com::send(serial_packet_t((uint8_t *)&buff[1], 3));
    }
}

/*void sendStateStopARTDECO(byte* buff, int count){
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

void sendStateStopARTDECO(byte *buff, int count)
{
    if (count >= 3)
    {
        int index = buff[2] - '1';
        if (index < 0 || index > 3)
        {
            return;
        }
        byte buffOut[4] = {3, 'K', buff[2], digitalRead(stopStatesARTDECO[index]) + '0'};
        Com::send(serial_packet_t((uint8_t *)&buffOut[1], 3));
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            byte buff[4] = {3, 'K', i + '1', digitalRead(stopStatesARTDECO[i]) + '0'};
            Com::send(serial_packet_t((uint8_t *)&buff[1], 3));
        }
    }
}

void verificationStopsArdko()
{
    if ((unsigned long)(millis() - previousStopsArdkoTime) >= 100)
    {
        previousStopsArdkoTime = millis();
        for (int i = 0; i < 4; ++i)
        {
            boolean state = digitalRead(stopStatesARTDECO[i]);
            if (state != StatesArtDeco[i])
            {
                byte buff[4] = {3, 'K', (i + 1) + '0', state + '0'};
                Com::send(serial_packet_t((uint8_t *)&buff[1], 3));
                StatesArtDeco[i] = state;
            }
        }
    }
}