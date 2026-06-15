#include "DueStepper.hpp"

DueStepper::DueStepper(uint8_t timerId, due_stepper_pins_t pins) :
    m_timer(timerId),
    m_pins(pins)
{
}

bool DueStepper::setup(uint32_t initialFrequency)
{
    pinMode(m_pins.enable, OUTPUT);
    pinMode(m_pins.direction, OUTPUT);
    pinMode(m_pins.step, OUTPUT);
    pinMode(m_pins.resolution[0], OUTPUT);
    pinMode(m_pins.resolution[1], OUTPUT);

    this->enable(false);
    digitalWrite(m_pins.step, LOW);

    m_frequency = initialFrequency;                                     // Storing steps per second
    return m_timer.begin(m_frequency * 2, &DueStepper::isrPulse, this); // Motor steps on rising edge only so multiply by 2;
}

bool DueStepper::setTimerNVICPriority(uint8_t nvic_priority)
{
    if (m_timer.isInitialized())
    {
        m_timer.setNVICPriority(nvic_priority);
        return true;
    }
    return false;
}

void DueStepper::start(void)
{
    this->enable(true);
    m_timer.start();
}

void DueStepper::stop(MotorStopReason r)
{
    this->enable(false);
    m_timer.stop();

    if (m_onStop)
        m_onStop(r);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    m_stepTargetSet    = false;
    m_encoderTargetSet = false;
    __set_PRIMASK(primask);
}

void DueStepper::enable(bool en)
{
    digitalWrite(m_pins.enable, en ? LOW : HIGH); // TODO: active high or low ?
}

void DueStepper::attachEncoder(volatile int32_t *encoderCountPtr)
{
    m_encoderCountPtr = encoderCountPtr;
}

void DueStepper::attachOnStopCallback(void (*cb)(MotorStopReason reason))
{
    m_onStop = cb;
}

static bool resolutionIsValid(uint8_t frac)
{
    return frac == StepFraction::NONE ||
           frac == StepFraction::FRAC_2 ||
           frac == StepFraction::FRAC_4 ||
           frac == StepFraction::FRAC_8 ||
           frac == StepFraction::FRAC_16 ||
           frac == StepFraction::FRAC_32;
}

void DueStepper::setStepFraction(uint32_t stepFraction)
{
    if (!resolutionIsValid(stepFraction))
        return;

    m_stepFraction = static_cast<StepFraction>(stepFraction);
    if (m_pins.resolutionSetter)
        m_pins.resolutionSetter(m_stepFraction, m_pins.resolution);
}

void DueStepper::setDirection(uint8_t dir)
{
    m_direction = dir ? HIGH : LOW;
    digitalWrite(m_pins.direction, m_direction);
}

void DueStepper::setFrequency(uint32_t hz)
{
    m_frequency = hz;             // Storing steps per second
    m_timer.setFrequency(hz * 2); // Motor steps on rising edge only so multiply by 2
}

void DueStepper::setStepTarget(uint32_t stepTarget)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    m_stepTarget       = stepTarget;
    m_stepTargetSet    = (m_stepTarget > 0);
    m_encoderTargetSet = false; // Mutually exclusive

    __set_PRIMASK(primask);
}

void DueStepper::setEncoderTarget(int32_t targetPosition)
{
    if (!m_encoderCountPtr)
        return;

    // Disable interrupts to guarantee multi-variable atomicity.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    int32_t currentCount = *m_encoderCountPtr;
    int32_t diffToTarget = abs(targetPosition - currentCount);

    m_encoderTarget = targetPosition;
    m_stepTargetSet = false; // Mutually exclusive movement mode: disable step target

    if (diffToTarget > ENCODER_TARGET_TOLERANCE)
    {
        // Determine which direction we need to cross the threshold
        if (targetPosition > currentCount)
        {
            m_encoderTargetDirection  = 1;
            m_encoderFailsafeBoundary = currentCount - ENCODER_BACKLASH_TOLERANCE;
        }
        else if (targetPosition < currentCount)
        {
            m_encoderTargetDirection  = -1;
            m_encoderFailsafeBoundary = currentCount + ENCODER_BACKLASH_TOLERANCE;
        }
        m_encoderTargetSet = true;
    }
    else // Current position in tolerance range
    {
        m_encoderTargetSet = false;
        this->stop(MotorStopReason::EncoderPositionReached);
    }

    __set_PRIMASK(primask);
}

uint8_t DueStepper::direction(void) const
{
    return m_direction;
}

bool DueStepper::running(void) const
{
    return m_timer.enabled();
}

void DueStepper::clearTargets(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    m_stepTargetSet    = false;
    m_encoderTargetSet = false;

    __set_PRIMASK(primask);
}

void DueStepper::pulse(void)
{
    m_prevStepState = !m_prevStepState;
    digitalWrite(m_pins.step, m_prevStepState); // Toggles pin state

    // Process target logic when falling edge detected (full stepper cycle completed)
    if (m_prevStepState == LOW)
    {
        // Mode 1: Moving a fixed number of steps
        if (m_stepTargetSet)
        {
            if (m_stepTarget > 0)
                --m_stepTarget;

            if (m_stepTarget == 0)
                this->stop(MotorStopReason::StepTargetReached);
        }
        // Mode 2: Moving to a specific encoder position
        else if (m_encoderTargetSet && m_encoderCountPtr != nullptr)
        {
            const int32_t currentCount        = *m_encoderCountPtr;
            const uint32_t absDiffToTargetPos = abs(currentCount - m_encoderTarget);

            // Positive direction
            if (m_encoderTargetDirection > 0 &&
                (absDiffToTargetPos < ENCODER_TARGET_TOLERANCE || // Target reached
                 currentCount >= m_encoderTarget ||               // Target exceeded
                 currentCount < m_encoderFailsafeBoundary))       // Invalid direction (Failsafe mechanism)
            {
                this->stop((absDiffToTargetPos < ENCODER_TARGET_TOLERANCE) ? MotorStopReason::EncoderPositionReached : MotorStopReason::TargetExceeded);
            }
            // Negative direction
            else if (m_encoderTargetDirection < 0 &&
                     (absDiffToTargetPos < ENCODER_TARGET_TOLERANCE || // Target reached
                      currentCount <= m_encoderTarget ||               // Target exceeded
                      currentCount > m_encoderFailsafeBoundary))       // Invalid direction (Failsafe mechanism)
            {
                this->stop((absDiffToTargetPos < ENCODER_TARGET_TOLERANCE) ? MotorStopReason::EncoderPositionReached : MotorStopReason::TargetExceeded);
            }
        }
    }
}

void DueStepper::isrPulse(void *stepperInstance)
{
    if (!stepperInstance)
        return;

    static_cast<DueStepper *>(stepperInstance)->pulse();
}
