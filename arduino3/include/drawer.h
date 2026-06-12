#ifndef DRAWER__H
#define DRAWER__H

#include "DueStepper.hpp"

namespace Drawer
{
    void setup(void);
    void loop(void);

    // Controllers
    void startMotor(char *, int);
    void moveToEncoderPosition(char *, int);
    void stopMotor(uint8_t idx, MotorStopReason reason);
    void sendAllStopsStatus(void);
}

#endif // DRAWER__H
