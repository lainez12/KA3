#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

#include "DueStepper.hpp"

#define MOTORS_COUNT     7
#define LIMIT_PINS_COUNT 10

namespace Motors
{
    void setup(void);
    void loop(void);

    // Controllers
    void startMotor(char *, int);
    void moveToEncoderPosition(char *, int);
    void stopMotor(uint8_t, MotorStopReason);
    void sendAllLimitsValues(void);
    void enableMotor(char *, int);
    void lockAlignment(void);
    void unlockAlignment(void);
}

#endif // MOTORS_H
