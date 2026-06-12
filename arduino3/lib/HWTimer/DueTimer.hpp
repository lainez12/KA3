#ifndef DUE_TIMER_H
#define DUE_TIMER_H

#include <Arduino.h>

/**
 * @class DueTimer
 * @brief Handles bare-metal hardware timers on the Arduino Due (SAM3X8E).
 *
 * The Arduino Due has 9 hardware timer channels:
 * - Timer 0: TC0 Channel 0 (TC0_IRQn)
 * - Timer 1: TC0 Channel 1 (TC1_IRQn)
 * - Timer 2: TC0 Channel 2 (TC2_IRQn)
 * - Timer 3: TC1 Channel 0 (TC3_IRQn)
 * - Timer 4: TC1 Channel 1 (TC4_IRQn)
 * - Timer 5: TC1 Channel 2 (TC5_IRQn)
 * - Timer 6: TC2 Channel 0 (TC6_IRQn)
 * - Timer 7: TC2 Channel 1 (TC7_IRQn)
 * - Timer 8: TC2 Channel 2 (TC8_IRQn)
 */
class DueTimer
{
public:
    // Instantiate with a timer ID from 0 to 8
    DueTimer(uint8_t timerId);
    ~DueTimer();

    // Initialize timer with a specific frequency and callback
    bool begin(uint32_t hz, void (*callback)(void *), void *context);
    // Update frequency on the fly without stopping the timer
    void setFrequency(uint32_t hz);
    // Start / Resume the timer
    void start(void);
    // Stop / Pause the timer
    void stop(void);
    // Free the hardware timer so another DueTimer instance can use it
    void freeTimer(void);
    // Change or attach the interrupt callback manually
    void attachInterrupt(void (*callback)(void *), void *context);
    // Remove the interrupt callback
    void detachInterrupt(void);
    // Sets the timer NVIC priority
    void setNVICPriority(uint8_t priority);
    // Returns whether the timer was succesfully hardware-initialized (registers)
    bool isInitialized(void) const;
    // Returns whether the timer is currently active (enabled)
    bool enabled(void) const;

private:
    void _initHardware(void);

private:
    bool m_hwInitialized = false;
    bool m_isEnabled     = false;
    uint8_t _id;
    Tc *_tc;
    uint8_t _channel;
    IRQn_Type _irq;
    uint32_t _pmcId;
};

#endif // DUE_TIMER_H
