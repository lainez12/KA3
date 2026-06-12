#ifndef ENCODER_H
#define ENCODER_H

#define ENCODERS_COUNT 5

enum MotorId
{
    Z_LEFT       = '1',
    Z_RIGHT      = '2',
    Z_BACK       = '3',
    MASK_DRAWER  = '4',
    WAFER_DRAWER = '5',
};

namespace Encoders
{
    void setup();

    // Senders

    void sendAll();
    void sendChanged();

    // Updaters

    void resetCount(char *buff, int count);

    // Getters

    // Returns a pointer to the volatile memory of a specific encoder count.
    // This allows external hardware timers to read the count atomically (1-cycle LDR).
    volatile int32_t *getCountPtr(MotorId id);
    int32_t getValue(MotorId id);
}

#endif