/*
 *  Arduino 1 (Kub3.8i) firmware
 */

#include <Arduino.h>

#include "DueStepper.hpp"
#include "SerialTXHandler.h"
#include "definitions.h"
#include "encoder.h"
#include "focalLed.h"
#include "motors.h"
#include "utils.h"

void processInstruction(char *buff, uint32_t size);

void setup()
{
    Serial.begin(115200);

    // Announce identity natively using TX handler
    Com::send(serial_packet_t((uint8_t *)VERSION, 20));

    FocalLed::setup();
    Encoders::setup();
    Motors::setup();
}

void loop()
{
    Motors::loop();
    Encoders::loop();
    Com::processPackets();
}

void orderStopMotor(char *buff, int count)
{
    if (count < 2)
        return;

    const uint8_t idx = KUtils::asciiByteToMotorIndex(buff[1]);
    if (idx == INVALID_MOTOR_INDEX)
        return; // Invalid index

    Motors::stopMotor(idx, MotorStopReason::SoftwareOrder);
}

// ---------------------------------------------------------------------
// Communication Core
// ---------------------------------------------------------------------

void serialEvent()
{
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
        orderStopMotor(buff, size);
        break;
    case '2':
        Motors::startMotor(buff, size);
        break;
    case '3':
        Motors::enableMotor(buff, size);
        break;
    case '4':
        FocalLed::sendSPI(buff, size);
        break;
    case '5':
        FocalLed::disable(buff, size);
        break;
    case '7':
        Motors::unlockAlignment();
        break;
    case '8':
        Motors::lockAlignment();
        break;
    case 'R':
        Encoders::resetCount(buff, size);
        break;
    case 'T':
        Motors::moveToEncoderPosition(buff, size);
        break;
    case '?':
    {
        if (size > 1)
        {
            if (buff[1] == 'S')
                Motors::sendAllLimitsValues();
            else if (buff[1] == 'C')
                Encoders::sendAll();
        }
        else // Request for version
        {
            Com::send(serial_packet_t((uint8_t *)VERSION, 21));
        }
        break;
    }
    default:
        break;
    }
}
