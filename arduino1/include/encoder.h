#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#define ENCODERS_COUNT 7

namespace Encoders
{
    /**
     * @brief Configures bare-metal pins as inputs and attaches hardware EXTI
     * interrupts for high-frequency quadrature decoding.
     */
    void setup();

    /**
     * @brief Periodic state machine tick. Evaluates if encoder changes need
     * to be flushed to the UART DMA queue.
     */
    void loop();

    /**
     * @brief Forces a transmission of all 7 encoder counts over Serial.
     */
    void sendAll();

    /**
     * @brief Transmits only the encoders that have seen a physical state change
     * since the last transmission. Clears the change flags atomically.
     */
    void sendChanged();

    /**
     * @brief Overrides the current count of a specific encoder (e.g., for homing).
     * Parses the ID and 32-bit position natively from the incoming serial buffer.
     *
     * @param buff Pointer to the raw serial payload.
     * @param count Total byte length of the payload.
     */
    void resetCount(char *buff, int count);

    /**
     * @brief Retrieves a raw memory pointer to a specific encoder's 32-bit count.
     * Essential for DueStepper to perform zero-overhead, closed-loop targeting.
     */
    volatile int32_t *getCountPtr(uint8_t idx);

    /**
     * @brief Retrieves the current step count of a specific encoder.
     */
    int32_t getValue(uint8_t idx);
}

#endif // ENCODER_H
