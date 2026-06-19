#ifndef DUE_TIMER_PWM_H
#define DUE_TIMER_PWM_H

#include <Arduino.h>

/**
 * @class DueTimerPWM
 * @brief Generate PWM signals using hardware via the Timers (TC) blocks.
 * Ideal for Due pins that don't have native PWM support, but 
 * if they are connected to timer outputs (e.g., pins 2, 3, 4, 5, 10, 11, 12, 13).
 */
class DueTimerPWM
{
public:
    DueTimerPWM(uint8_t arduinoPin);
    
    // Start the timer to generate continuous hardware-based PWM at the desired frequency
    bool begin(uint32_t frequency, uint32_t maxResolution = 4095);
    
    // Change the Duty Cycle (0 to maxResolution)
    void setDuty(uint32_t dutyValue);
    
    // Turn off the PWM
    void stop();

private:
    uint8_t _pin;
    uint32_t _maxResolution;
    bool _isTIOA; // A or B output of the timer channel
    
    // Pointers to the hardware for fast access
    TcChannel* _tcChannel;
};

#endif // DUE_TIMER_PWM_H