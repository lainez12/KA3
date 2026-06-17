#ifndef DUE_PWM_H
#define DUE_PWM_H

#include <Arduino.h>

/**
 * @class DuePWM
 * @brief It uses the Arduino Due's dedicated hardware PWM controller.
 * * Unlike Timers (TC), this block generates the duty cycle signal
 * automatically on the pins without consuming CPU cycles through interrupts.
 */
class DuePWM
{
public:
    // Instance passing the Arduino's physical PIN (ej. 7, 8, 9)
    DuePWM(uint8_t arduinoPin);
    
    // Configure the base frequency of the PWM (ej. 20000 for 20kHz, ideal for DC motors)
    // and the maximum resolution value (ej. 255, 1000, 4095)
    bool begin(uint32_t frequency, uint32_t maxResolution = 1000);
    
    // Change the motor speed in real time. 
    // If maxResolution is 1000, setDuty(500) is 50% speed.
    void setDuty(uint32_t dutyValue);
    
    // Stop the PWM leaving the pin in LOW
    void stop();

private:
    uint8_t _pin;
    uint32_t _resolution;
    uint32_t _channel;
};

#endif // DUE_PWM_H