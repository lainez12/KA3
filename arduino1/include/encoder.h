#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#define ENCODERS_COUNT 7

namespace Encoders
{
    void setup();
    void loop();

    void sendAll();
    void sendChanged();
    void resetCount(char *buff, int count);

    volatile int32_t *getCountPtr(uint8_t idx);
    int32_t getValue(uint8_t idx);
}

#endif // ENCODER_H
