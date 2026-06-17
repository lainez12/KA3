#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#define ENCODERS_COUNT 5

/**
 * @brief Zero-indexed enum mapping exactly to the 5 motors across
 * the Autolevel and Drawer systems.
 */
enum MotorId : uint8_t
{
    Z_LEFT       = 0,
    Z_RIGHT      = 1,
    Z_BACK       = 2,
    MASK_DRAWER  = 3,
    WAFER_DRAWER = 4,
    MOTORS_COUNT = 5
};

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
    // -------------------------------------------------------------------------
    // Senders (UART DMA)
    // -------------------------------------------------------------------------

    /**
     * @brief Forces a transmission of all 5 encoder counts over Serial.
     */
    void sendAll();

    /**
     * @brief Transmits only the encoders that have seen a physical state change
     * since the last transmission. Clears the change flags atomically.
     */
    void sendChanged();

    // -------------------------------------------------------------------------
    // Updaters
    // -------------------------------------------------------------------------

    /**
     * @brief Overrides the current count of a specific encoder (e.g., for initialization).
     * Parses the ID and 32-bit position natively from the incoming serial buffer.
     *
     * @param buff Pointer to the raw serial payload.
     * @param count Total byte length of the payload.
     */
    void resetCount(char *buff, int count);

    // -------------------------------------------------------------------------
    // Getters
    // -------------------------------------------------------------------------

    /**
     * @brief Retrieves a raw memory pointer to a specific encoder's 32-bit count.
     * Essential for DueStepper to perform zero-overhead, closed-loop targeting
     * directly inside the Timer/Counter hardware ISR.
     *
     * @param id The `MotorId` associated with the encoder.
     * @return `volatile int32_t*` Pointer to the volatile memory, or nullptr if out of bounds.
     */
    volatile int32_t *getCountPtr(MotorId id);

    /**
     * @brief Retrieves the current step count of a specific encoder.
     *
     * @param id The `MotorId` associated with the encoder.
     * @return `int32_t` The current position, or `INT32_MIN` if the ID is invalid.
     */
    int32_t getValue(MotorId id);
}

#endif // ENCODER_H
