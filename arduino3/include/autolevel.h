#ifndef AUTOLEVEL_H
#define AUTOLEVEL_H

#include "DueStepper.hpp"

namespace Autolevel
{
    void setup(void);
    void loop(void);
    // Motors
    void startMotor(char *, int);
    void stopMotor(uint8_t, MotorStopReason);
    void moveToEncoderPosition(char *, int);
    // Stops
    void checkAllAutolevelStopPins(void);
    void checkAllMaskingZoneStopPins(void);
    void sendAllAutolevelStopsStatus(void);
    void sendAllMaskingZoneStopsStatus(void);
}

#endif // AUTOLEVEL_H
