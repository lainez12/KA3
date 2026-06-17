#include "DueDCMotor.hpp"

DueDCMotor::DueDCMotor(due_dc_motor_pins_t pins) :
    m_pwm(pins.pwm), // Initialize the DuePWM class with the corresponding pin
    m_pins(pins),
    m_direction(LOW),
    m_currentSpeed(0),
    m_enabled(false),
    m_maxResolution(1000)
{
}

bool DueDCMotor::setup(uint32_t frequency, uint32_t maxResolution)
{
    pinMode(m_pins.disable, OUTPUT);
    pinMode(m_pins.direction, OUTPUT);
    pinMode(m_pins.torque, INPUT);

    m_maxResolution = maxResolution;

    this->enable(false);
    this->setDirection(LOW);

    return m_pwm.begin(frequency, m_maxResolution);
}

void DueDCMotor::enable(bool en)
{
    m_enabled = en;
    digitalWrite(m_pins.disable, en ? LOW : HIGH); 
}

void DueDCMotor::stop(void)
{
    this->setSpeed(0);
    this->enable(false);
}

void DueDCMotor::setDirection(uint8_t dir)
{
    m_direction = dir ? HIGH : LOW;
    digitalWrite(m_pins.direction, m_direction);
}

void DueDCMotor::setSpeed(uint32_t speed)
{
    // Safety clamping to prevent exceeding the maximum resolution
    if (speed > m_maxResolution) {
        speed = m_maxResolution;
    }

    m_currentSpeed = speed;
    
    m_pwm.setDuty(m_currentSpeed); 
}

uint8_t DueDCMotor::direction(void) const
{
    return m_direction;
}

bool DueDCMotor::isEnabled(void) const
{
    return m_enabled;
}

uint32_t DueDCMotor::currentSpeed(void) const
{
    return m_currentSpeed;
}

uint32_t DueDCMotor::readTorque(void) const
{
    return analogRead(m_pins.torque);
}