#ifndef KLOE_DUE_STEPPER_H
#define KLOE_DUE_STEPPER_H

#include <Arduino.h>

#include "DueTimer.hpp"

#define ENCODER_BACKLASH_TOLERANCE 50 // TODO: adapt value
#define ENCODER_TARGET_TOLERANCE   3

enum StepFraction
{
    NONE    = 1,
    FRAC_2  = 2,
    FRAC_4  = 4,
    FRAC_8  = 8,
    FRAC_16 = 16,
    FRAC_32 = 32,
};

enum MotorStopReason
{
    SoftwareOrder          = 0,
    StepTargetReached      = 1,
    EncoderPositionReached = 2,
    LimitStopReached       = 3,
    TargetExceeded         = 4
};

typedef struct due_stepper_pins_s {
    uint32_t enable;
    uint32_t step;
    uint32_t direction;
    uint32_t *resolution;
    void (*resolutionSetter)(StepFraction frac, uint32_t *resolutionPins);
} due_stepper_pins_t;

class DueStepper
{
public:
    DueStepper(uint8_t timerId, due_stepper_pins_t pins);
    ~DueStepper() = default;

    bool setup(uint32_t initialFrequency = 1);
    void start(void);
    void stop(MotorStopReason r);

public:
    void enable(bool en); // (e.g. Enable motor to keep torque)
    void attachEncoder(volatile int32_t *encoderCountPtr);
    void attachOnStopCallback(void (*cb)(MotorStopReason reason));
    // Setters
    bool setTimerNVICPriority(uint8_t nvic_priority);
    void setStepFraction(uint32_t frac);
    void setDirection(uint8_t dir);
    void setFrequency(uint32_t hz);
    void setStepTarget(uint32_t stepTarget);
    void setEncoderTarget(int32_t targetPosition);
    // Getters
    uint8_t direction(void) const;
    bool running(void) const;
    // Clearers
    void clearTargets(void);

private:
    void pulse(void);
    static void isrPulse(void *stepperInstance);

private:
    // CTOR initialized
    DueTimer m_timer;
    const due_stepper_pins_t m_pins;
    void (*m_onStop)(MotorStopReason reason);

    // Internal state
    StepFraction m_stepFraction = StepFraction::NONE;
    uint8_t m_direction         = 0;
    uint32_t m_frequency        = 0;

    // --- ISR-used variables
    volatile uint8_t m_prevStepState = 0;
    // Steps target State
    volatile bool m_stepTargetSet  = false;
    volatile uint32_t m_stepTarget = 0;
    // Encoder target State
    volatile int32_t *m_encoderCountPtr        = nullptr;
    volatile int32_t m_encoderTarget           = 0;
    volatile bool m_encoderTargetSet           = false;
    volatile int8_t m_encoderTargetDirection   = 0; // 1 (forward), -1 (backward), 0 (arrived)
    volatile int32_t m_encoderFailsafeBoundary = 0;
};

#endif // KLOE_DUE_STEPPER_H
