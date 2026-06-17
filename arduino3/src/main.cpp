/*
 *  Arduino 3 (Kub3.8i) firmware
 */

#include <SPI.h>
#include <string.h>

#include "SerialTXHandler.h"
#include "definitions.h"
#include "encoder.h"
#include "forceSensors.h"
#include "motors.h"
#include "pins.h"

void processInstruction(char *buff, uint32_t size);

void setup()
{
    Serial.begin(115200);
    analogReadResolution(12); // Specific to Arduino3's Force Sensors

    // Announce identity on startup
    Com::send(serial_packet_t((uint8_t *)VERSION, VERSION_SIZE));

    Encoders::setup();
    ForceSensors::setup();
    Motors::setup();
}

void loop()
{
    Motors::loop();
    Encoders::loop();
    ForceSensors::loop();

    Com::processPackets();
}

// ---------------------------------------------------------------------
// Communication Core (Non-Blocking State Machine)
// ---------------------------------------------------------------------

void serialEvent()
{
    // Static state machine variables to parse incoming UART packets asynchronously.
    // This entirely replaces blocking calls like Serial.readBytes(), ensuring the
    // main loop (and polling tasks) never stall while waiting for a packet to finish arriving.
    static bool rxReading        = false; // false: Waiting for Length, true: Reading Data
    static uint8_t rxExpectedLen = 0;     // Number of bytes to read
    static int rxIndex           = 0;     // Current buffer position
    static char rxBuffer[MAXIMUM_SERIAL_INSTRUCTION_SIZE];

    while (Serial.available() > 0)
    {
        byte incoming = Serial.read();

        if (!rxReading)
        {
            rxExpectedLen = incoming;
            rxIndex       = 0;

            if (rxExpectedLen > 0 && rxExpectedLen < MAXIMUM_SERIAL_INSTRUCTION_SIZE)
                rxReading = true;
        }
        else
        {
            rxBuffer[rxIndex++] = (char)incoming;

            if (rxIndex >= rxExpectedLen)
            {
                processInstruction(rxBuffer, rxExpectedLen);
                rxReading = false;
            }
        }
    }
}

void processInstruction(char *buff, uint32_t size)
{
    if (size == 0)
        return;

    switch (buff[0])
    {
    case '1':
        if (size > 1)
            Motors::stopMotorCommand(buff, size);
        break;
    case '2':
        Motors::startMotor(buff, size);
        break;
    case 'R':
        Encoders::resetCount(buff, size);
        break;
    case 'T':
        Motors::moveToEncoderPosition(buff, size);
        break;
    case 'F':
        ForceSensors::setEnabledState(buff, size);
        break;
    case '?':
    {
        if (size > 1)
        {
            switch (buff[1])
            {
            case 'S':
                Motors::sendAllStdLimitsStatus();
                break;
            case 'C':
                Encoders::sendAll();
                break;
            case 'F':
                ForceSensors::sendEnabledState(buff, size);
                break;
            case 'Z':
                Motors::sendAllZLimitsStatus();
                break;
            default:
                break;
            }
        }
        else // Request for version ('?')
        {
            Com::send(serial_packet_t((uint8_t *)VERSION, 20));
        }
        break;
    }
    default:
        break;
    }
}
