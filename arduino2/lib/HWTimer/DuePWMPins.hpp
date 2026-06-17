#ifndef DUE_PWM_PINS_H
#define DUE_PWM_PINS_H

#include <Arduino.h>

struct HW_PWM_PinInfo
{
    uint8_t arduinoPin;
    Pio *pioPort;
    uint32_t pinMask;
    uint32_t peripheral;
    uint8_t channel;
};

static const HW_PWM_PinInfo due_hw_pwm_pins[] = {
    // Pins Arduino | Ports | Mask Pin | Periférico MUX | Channel PWM
    {6, PIOC, PIO_PC24, PIO_ABSR_P24, 7}, // PWML7
    {7, PIOC, PIO_PC23, PIO_ABSR_P23, 6}, // PWML6
    {8, PIOC, PIO_PC22, PIO_ABSR_P22, 5}, // PWML5
    {9, PIOC, PIO_PC21, PIO_ABSR_P21, 4}, // PWML4
    {34, PIOC, PIO_PC2, PIO_ABSR_P2, 0},  // PWML0 (Digital 34)
    {36, PIOC, PIO_PC4, PIO_ABSR_P4, 1},  // PWML1 (Digital 36)
    {38, PIOC, PIO_PC6, PIO_ABSR_P6, 2},  // PWML2 (Digital 38)
    {40, PIOC, PIO_PC8, PIO_ABSR_P8, 3}   // PWML3 (Digital 40)
};

static const uint8_t NUM_HW_PWM_PINS = sizeof(due_hw_pwm_pins) / sizeof(HW_PWM_PinInfo);

#endif // DUE_PWM_PINS_H