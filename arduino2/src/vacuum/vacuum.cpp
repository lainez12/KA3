#include "SerialTXHandler.h"
#include "vacuum.h"

static unsigned long previousStopsVacuumsTime   = 0;
static unsigned int compressedAirSensorPinState = HIGH;
static unsigned int pinVacuums[2]               = {SM_VACUUM, SW_VACUUM};
static unsigned int statesVacuum[2]             = {HIGH, HIGH};

void setupVacuumsensor()
{
    // TODO review
    pinMode(SM_VACUUM, INPUT);
    pinMode(SW_VACUUM, INPUT);
    pinMode(SW_SENSOR_COMPRESSED_AIR, INPUT);
}

void sendStateSensor(char *buff, int count)
{
    // TODO review
    if (count != 4)
    {
        return;
    }
    // "?VC" + <IDCapteur> (1 octét ascii)
    if ((buff[3] == 'M') || (buff[3] == 'm'))
    {
        char state        = digitalRead(SM_VACUUM);
        uint8_t stateByte = !state + '0';
        byte buffTemp[5]  = {4, 'V', 'C', buff[3], stateByte};
        Com::send(serial_packet_t((uint8_t *)&buffTemp[1], 4));
    }
    else if ((buff[3] == 'W') || (buff[3] == 'w'))
    {
        char state        = digitalRead(SW_VACUUM);
        uint8_t stateByte = !state + '0';
        byte buffTemp[5]  = {4, 'V', 'C', buff[3], stateByte};
        Com::send(serial_packet_t((uint8_t *)&buffTemp[1], 4));
    }
}

void verificationStatesVacuum()
{
    if ((unsigned long)(millis() - previousStopsVacuumsTime) >= 100)
    {
        previousStopsVacuumsTime = millis();
        for (int i = 0; i < 2; ++i)
        {
            boolean state = digitalRead(pinVacuums[i]);
            if (state != statesVacuum[i])
            {
                statesVacuum[i]   = state;
                uint8_t stateByte = !state + '0';
                byte buff[5]      = {4, 'V', 'C', (i) ? 'W' : 'M', stateByte};
                Com::send(serial_packet_t((uint8_t *)&buff[1], 4));
            }
        }

        // On vérifie également l'état de l'air comprimé
        const bool newCompressedAirSensorPinState = digitalRead(SW_SENSOR_COMPRESSED_AIR);

        if (newCompressedAirSensorPinState != compressedAirSensorPinState)
        {
            compressedAirSensorPinState = newCompressedAirSensorPinState; // Update stored value locally
            sendCompressedAirSensorState(newCompressedAirSensorPinState); // Send update through UART
        }
    }
}

void sendCompressedAirValveState(void)
{
    uint8_t compressedAirValveState = digitalRead(SW_COMPRESSED_AIR) + '0';             // state to ascii
    uint8_t buffTemp[6]             = {5, 'V', 'V', 'A', 'C', compressedAirValveState}; // Active high

    Com::send(serial_packet_t((uint8_t *)&buffTemp[1], 5));
}

void sendCompressedAirSensorState(void)
{
    sendCompressedAirSensorState(digitalRead(SW_SENSOR_COMPRESSED_AIR));
}

void sendCompressedAirSensorState(bool pinState)
{
    uint8_t stateByte = !pinState + '0';
    uint8_t buffer[5] = {4, 'V', 'A', 'C', stateByte}; // Active low => invert pin state

    Com::send(serial_packet_t((uint8_t *)&buffer[1], 4));
}
