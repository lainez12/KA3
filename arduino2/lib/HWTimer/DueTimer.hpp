#ifndef DUE_TIMER_H
#define DUE_TIMER_H

#include <Arduino.h>

enum class DueTimerMode
{
    UNINITIALIZED,
    INTERRUPT,
    PULSE_WIDTH_MODULATION
};

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

    // -------------------------------------------------------------------------
    // STATIC UTILITIES
    // -------------------------------------------------------------------------
    // Get the hardware Timer ID associated with an Arduino pin (for PWM)
    // Safe for Due pins: 2, 3, 4, 5, 10, 11, 12, 13
    static int8_t getTimerIdForPin(uint8_t pin);

    // -------------------------------------------------------------------------
    // INTERRUPT MODE (E.g. Steppers)
    // -------------------------------------------------------------------------
    // Initialize timer with a specific frequency and callback
    bool begin(uint32_t hz, void (*callback)(void *), void *context);
    // Change or attach the interrupt callback manually
    void attachInterrupt(void (*callback)(void *), void *context);
    // Remove the interrupt callback
    void detachInterrupt(void);
    // Sets the timer NVIC priority
    void setNVICPriority(uint8_t priority);

    // -------------------------------------------------------------------------
    // PWM MODE (E.g. DC Motors)
    // -------------------------------------------------------------------------
    // Initialize the timer for PWM mode at a specific frequency
    bool beginPWM(uint32_t frequency);
    // Attach a pin to this timer's PWM generation
    bool attachPWMPin(uint8_t pin);
    // Set the duty cycle for an attached PWM pin
    void setPWMDuty(uint8_t pin, uint32_t dutyValue, uint32_t maxResolution = 4095);
    // Stop PWM output on a specific pin (safely forces LOW)
    void stopPWMPin(uint8_t pin);

    // -------------------------------------------------------------------------
    // COMMON API
    // -------------------------------------------------------------------------
    // Update frequency on the fly without stopping the timer (adapts to mode)
    void setFrequency(uint32_t hz);
    // Start / Resume the timer
    void start(void);
    // Stop / Pause the timer
    void stop(void);
    // Free the hardware timer so another DueTimer instance can use it
    void freeTimer(void);
    // Returns whether the timer was succesfully hardware-initialized (registers)
    bool isInitialized(void) const;
    // Returns whether the timer is currently active (enabled)
    bool enabled(void) const;
    // Returns the current mode of the timer
    DueTimerMode getMode(void) const;

private:
    void _initHardwareInterrupt(void);
    void _initHardwarePWM(void);

private:
    // Struct to store hardware state for GPIO hijacking (0% / 100% states)
    struct PWMPinState {
        bool attached            = false;
        uint8_t pin              = 0;
        Pio *pioPort             = nullptr;
        uint32_t pinMask         = 0;
        _EPioType peripheralType = PIO_PERIPH_B;
    };

    DueTimerMode m_mode  = DueTimerMode::UNINITIALIZED;
    bool m_hwInitialized = false;
    bool m_isEnabled     = false;

    uint8_t _id;
    Tc *_tc;
    uint8_t _channel;
    IRQn_Type _irq;
    uint32_t _pmcId;

    // TIOA and TIOB logic states for PWM Mode
    PWMPinState m_pwmA;
    PWMPinState m_pwmB;
};

#endif // DUE_TIMER_H