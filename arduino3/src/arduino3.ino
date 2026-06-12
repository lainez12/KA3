/*
 *	Programme pour Arduino3 Kub3.8i Dec (programming port ttyACM#)
 *	Revision 8p v1.15 (23/12/2025)
 */

#include <SPI.h>
#include <string.h>

#include "autolevel.h"
#include "drawer.h"
#include "encoder.h"
#include "forceSensors.h"
#include "pins.h"
#include "serialTxHandler.h"

#define VERSION "? : Arduino3 8p v1.15"

#define NB_MOTORS_AL 3u

uint32_t prevCoderTime  = 0;
uint32_t prevSensorTime = 0;

void setup()
{
    Serial.begin(115200);
    analogReadResolution(12);

    Com::send(serial_packet_t((uint8_t *)VERSION, 21));

    Encoders::setup();
    ForceSensors::setup();
    Autolevel::setup();
    Drawer::setup();
}

void loop()
{
    uint32_t now = millis();

    // Every 10ms
    if (now - prevSensorTime >= 10)
    {
        Autolevel::checkAllAutolevelStopPins();
        Autolevel::checkAllMaskingZoneStopPins();
        prevSensorTime = now;
    }

    now = millis();
    if (now - prevCoderTime >= 100)
    {
        // Send encoders value updates every 100ms
        Encoders::sendChanged();
        prevCoderTime = now;
    }

    Autolevel::loop();
    Drawer::loop();
    ForceSensors::loop();

    Com::processPackets();
}

void orderStopMotor(char *buff, int count)
{
    const uint8_t index = buff[1] - '1';

    if (index >= 5)
        return; // Invalid index

    if (index >= NB_MOTORS_AL)
        Drawer::stopMotor(index - NB_MOTORS_AL, MotorStopReason::SoftwareOrder);
    else
        Autolevel::stopMotor(index, MotorStopReason::SoftwareOrder);
}

void orderStartMotor(char *buff, int count)
{
    const uint8_t index = buff[1] - '1';

    if (index >= NB_MOTORS_AL)
        Drawer::startMotor(buff, count);
    else
        Autolevel::startMotor(buff, count);
}

void orderMoveToTarget(char *buff, int count)
{
    const uint8_t index = buff[1] - '1';

    if (index >= NB_MOTORS_AL)
        Drawer::moveToEncoderPosition(buff, count);
    else
        Autolevel::moveToEncoderPosition(buff, count);
}

void sendAllStops()
{
    Drawer::sendAllStopsStatus();
    Autolevel::sendAllAutolevelStopsStatus();
}

//  ---------------------------------------------------------------------
//                    Communication
//  ---------------------------------------------------------------------

void serialEvent()
{
    char buff[64];
    int nb    = int(Serial.read());
    int count = Serial.readBytes(buff, nb);

    if (count == nb)
    {
        if (count == 1 && buff[0] == '?')
        {
            Com::send(serial_packet_t((uint8_t *)VERSION, 21));
            return;
        }
        switch (buff[0])
        {
        case '1':
        {
            if (count > 1)
                orderStopMotor(buff, count);
            break;
        }
        case '2':
        {
            orderStartMotor(buff, count);
            break;
        }
        case 'R':
        {
            Encoders::resetCount(buff, count);
            break;
        }
        case 'T':
        {
            orderMoveToTarget(buff, count);
            break;
        }
        case 'F':
        {
            ForceSensors::setEnabledState(buff, count);
            break;
        }
        case '?':
        {
            if (count < 2)
                return;

            switch (buff[1])
            {
            case 'S':
                sendAllStops();
                break;
            case 'C':
                Encoders::sendAll();
                break;
            case 'F':
                ForceSensors::sendEnabledState(buff, count);
                break;
            case 'Z':
                Autolevel::sendAllMaskingZoneStopsStatus();
                break;
            default:
                break;
            }
            break;
        }
        default:
            break;
        }
    }
}
