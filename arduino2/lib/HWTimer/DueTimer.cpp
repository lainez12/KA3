#include "DueTimer.hpp"

// Global array to store the callbacks for all 9 timers
static void (*DueTimer_Callbacks[9])(void *) = {nullptr};
// Global array to store the callbacks contexts (passed as argument to the function)
static void *DueTimer_Contexts[9] = {nullptr};
// Resource management array to prevent multiple objects from hijacking the same timer
static DueTimer *DueTimer_Allocations[9] = {nullptr};

int8_t DueTimer::getTimerIdForPin(uint8_t pin)
{
    // Hardware Multiplexer Map for Arduino Due Timer Pins
    switch (pin)
    {
    case 2:
        return 0; // TC0 Ch 0 (TIOA)
    case 13:
        return 0; // TC0 Ch 0 (TIOB)
    case 4:
        return 6; // TC2 Ch 0 (TIOB)
    case 5:
        return 6; // TC2 Ch 0 (TIOA)
    case 3:
        return 7; // TC2 Ch 1 (TIOA)
    case 10:
        return 7; // TC2 Ch 1 (TIOB)
    case 11:
        return 8; // TC2 Ch 2 (TIOA)
    case 12:
        return 8; // TC2 Ch 2 (TIOB)
    default:
        return -1;
    }
}

DueTimer::DueTimer(uint8_t timerId)
{
    // Bound the ID between 0 and 8
    _id = (timerId > 8) ? 8 : timerId;

    // The SAM3X8E PMC IDs and IRQs for TC0 to TC8 are contiguous starting at 27
    _pmcId = ID_TC0 + _id;
    _irq   = (IRQn_Type)(TC0_IRQn + _id);

    // Map the 9 IDs to their respective hardware Timer Counters (TC0, TC1, TC2)
    if (_id < 3)
        _tc = TC0;
    else if (_id < 6)
        _tc = TC1;
    else
        _tc = TC2;

    // Channels repeat 0, 1, 2 for each TC block
    _channel = _id % 3;
}

DueTimer::~DueTimer()
{
    this->freeTimer();
}

bool DueTimer::begin(uint32_t hz, void (*callback)(void *), void *context)
{
    // Invalid frequency or timer already allocated AND not to THIS specific object instance
    if ((hz <= 0) || (DueTimer_Allocations[_id] != nullptr && DueTimer_Allocations[_id] != this))
        return false;

    DueTimer_Allocations[_id] = this;
    m_mode                    = DueTimerMode::INTERRUPT;

    this->attachInterrupt(callback, context);
    this->_initHardwareInterrupt();
    this->setFrequency(hz);

    return true;
}

bool DueTimer::beginPWM(uint32_t frequency)
{
    // Invalid frequency or timer already allocated AND not to THIS specific object instance
    if ((frequency <= 0) || (DueTimer_Allocations[_id] != nullptr && DueTimer_Allocations[_id] != this))
        return false;

    DueTimer_Allocations[_id] = this;
    m_mode                    = DueTimerMode::PULSE_WIDTH_MODULATION;

    this->_initHardwarePWM();
    this->setFrequency(frequency);

    return true;
}

bool DueTimer::attachPWMPin(uint8_t pin)
{
    if (m_mode != DueTimerMode::PULSE_WIDTH_MODULATION)
        return false;

    bool isTIOA              = false;
    Pio *pioPort             = nullptr;
    uint32_t pinMask         = 0;
    _EPioType peripheralType = PIO_PERIPH_B; // Most route to Periph B

    switch (pin)
    {
    case 2:
        if (_id != 0)
            return false;
        isTIOA         = true;
        pioPort        = PIOB;
        pinMask        = PIO_PB25;
        peripheralType = PIO_PERIPH_B;
        break;
    case 13:
        if (_id != 0)
            return false;
        isTIOA         = false;
        pioPort        = PIOB;
        pinMask        = PIO_PB27;
        peripheralType = PIO_PERIPH_B;
        break;
    case 4:
        if (_id != 6)
            return false;
        isTIOA         = false;
        pioPort        = PIOC;
        pinMask        = PIO_PC26;
        peripheralType = PIO_PERIPH_B;
        break;
    case 5:
        if (_id != 6)
            return false;
        isTIOA         = true;
        pioPort        = PIOC;
        pinMask        = PIO_PC25;
        peripheralType = PIO_PERIPH_B;
        break;
    case 3:
        if (_id != 7)
            return false;
        isTIOA         = true;
        pioPort        = PIOC;
        pinMask        = PIO_PC28;
        peripheralType = PIO_PERIPH_B;
        break;
    case 10:
        if (_id != 7)
            return false;
        isTIOA         = false;
        pioPort        = PIOC;
        pinMask        = PIO_PC29;
        peripheralType = PIO_PERIPH_B;
        break;
    case 11:
        if (_id != 8)
            return false;
        isTIOA         = true;
        pioPort        = PIOD;
        pinMask        = PIO_PD7;
        peripheralType = PIO_PERIPH_B;
        break;
    case 12:
        if (_id != 8)
            return false;
        isTIOA         = false;
        pioPort        = PIOD;
        pinMask        = PIO_PD8;
        peripheralType = PIO_PERIPH_B;
        break;
    default:
        return false;
    }

    if (isTIOA)
    {
        m_pwmA.attached                 = true;
        m_pwmA.pin                      = pin;
        m_pwmA.pioPort                  = pioPort;
        m_pwmA.pinMask                  = pinMask;
        m_pwmA.peripheralType           = peripheralType;
        _tc->TC_CHANNEL[_channel].TC_RA = 0; // Initialize duty cycle to 0
    }
    else
    {
        m_pwmB.attached                 = true;
        m_pwmB.pin                      = pin;
        m_pwmB.pioPort                  = pioPort;
        m_pwmB.pinMask                  = pinMask;
        m_pwmB.peripheralType           = peripheralType;
        _tc->TC_CHANNEL[_channel].TC_RB = 0; // Initialize duty cycle to 0
    }

    // Attach the pin to the Timer Peripheral initially
    PIO_Configure(pioPort, peripheralType, pinMask, PIO_DEFAULT);

    // Apply TIOA or TIOB Waveform logical configurations without resetting the timer
    uint32_t cmr = _tc->TC_CHANNEL[_channel].TC_CMR;
    if (isTIOA)
        cmr |= TC_CMR_ACPC_SET | TC_CMR_ACPA_CLEAR; // TIOA logic
    else
        cmr |= TC_CMR_BCPC_SET | TC_CMR_BCPB_CLEAR | TC_CMR_EEVT_XC0; // TIOB logic
    _tc->TC_CHANNEL[_channel].TC_CMR = cmr;

    return true;
}

void DueTimer::setPWMDuty(uint8_t pin, uint32_t dutyValue, uint32_t maxResolution)
{
    if (m_mode != DueTimerMode::PULSE_WIDTH_MODULATION)
        return;

    PWMPinState *state = nullptr;
    bool isTIOA        = false;

    if (m_pwmA.attached && m_pwmA.pin == pin)
    {
        state  = &m_pwmA;
        isTIOA = true;
    }
    else if (m_pwmB.attached && m_pwmB.pin == pin)
    {
        state  = &m_pwmB;
        isTIOA = false;
    }
    else
    {
        return; // Pin not attached
    }

    if (dutyValue > maxResolution)
        dutyValue = maxResolution;

    uint32_t rc = _tc->TC_CHANNEL[_channel].TC_RC;

    // GPIO Hijack to safely pull lines to rigid limits without Timer artifacts
    if (dutyValue == 0)
    {
        PIO_Configure(state->pioPort, PIO_OUTPUT_0, state->pinMask, PIO_DEFAULT);
        return;
    }

    if (dutyValue == maxResolution)
    {
        PIO_Configure(state->pioPort, PIO_OUTPUT_1, state->pinMask, PIO_DEFAULT);
        return;
    }

    uint32_t duty_register_value = (rc * dutyValue) / maxResolution;
    if (duty_register_value == 0)
        duty_register_value = 1;
    if (duty_register_value >= rc)
        duty_register_value = rc - 1;

    // Ensure we switch back to the correct peripheral mode if previously hijacked
    PIO_Configure(state->pioPort, state->peripheralType, state->pinMask, PIO_DEFAULT);

    if (isTIOA)
        _tc->TC_CHANNEL[_channel].TC_RA = duty_register_value;
    else
        _tc->TC_CHANNEL[_channel].TC_RB = duty_register_value;
}

void DueTimer::stopPWMPin(uint8_t pin)
{
    this->setPWMDuty(pin, 0); // Safely forces the pin low via GPIO
}

void DueTimer::freeTimer()
{
    if (DueTimer_Allocations[_id] == this)
    {
        this->stop();

        if (m_mode == DueTimerMode::INTERRUPT)
        {
            this->detachInterrupt();
            NVIC_DisableIRQ(_irq);
        }
        else if (m_mode == DueTimerMode::PULSE_WIDTH_MODULATION)
        {
            if (m_pwmA.attached)
                this->stopPWMPin(m_pwmA.pin);
            if (m_pwmB.attached)
                this->stopPWMPin(m_pwmB.pin);
            m_pwmA.attached = false;
            m_pwmB.attached = false;
        }

        // Disable Peripheral Clock to save power
        if (_pmcId < 32)
            PMC->PMC_PCDR0 = (1 << _pmcId);
        else
            PMC->PMC_PCDR1 = (1 << (_pmcId - 32));

        DueTimer_Allocations[_id] = nullptr;
        m_hwInitialized           = false;
        m_mode                    = DueTimerMode::UNINITIALIZED;
    }
}

void DueTimer::_initHardwareInterrupt()
{
    if (_pmcId < 32)
        PMC->PMC_PCER0 = (1 << _pmcId);
    else
        PMC->PMC_PCER1 = (1 << (_pmcId - 32));
    _tc->TC_CHANNEL[_channel].TC_CCR = TC_CCR_CLKDIS;
    _tc->TC_CHANNEL[_channel].TC_IDR = 0xFFFFFFFF;
    _tc->TC_CHANNEL[_channel].TC_SR; // Read to clear flags
    // TCCLKS_TIMER_CLOCK3 = Master Clock / 32
    _tc->TC_CHANNEL[_channel].TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK3;
    _tc->TC_CHANNEL[_channel].TC_IER = TC_IER_CPCS;

    NVIC_ClearPendingIRQ(_irq);
    NVIC_SetPriority(_irq, 0);
    NVIC_EnableIRQ(_irq);

    m_hwInitialized = true;
}

void DueTimer::_initHardwarePWM()
{
    if (_pmcId < 32)
        PMC->PMC_PCER0 = (1 << _pmcId);
    else
        PMC->PMC_PCER1 = (1 << (_pmcId - 32));
    _tc->TC_CHANNEL[_channel].TC_CCR = TC_CCR_CLKDIS;
    _tc->TC_CHANNEL[_channel].TC_IDR = 0xFFFFFFFF;
    // TCCLKS_TIMER_CLOCK1 = Master Clock / 2
    _tc->TC_CHANNEL[_channel].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;

    m_hwInitialized = true;
}

void DueTimer::setNVICPriority(uint8_t nvic_priority)
{
    NVIC_ClearPendingIRQ(_irq);
    NVIC_SetPriority(_irq, nvic_priority);
    NVIC_EnableIRQ(_irq);
}

bool DueTimer::isInitialized(void) const
{
    return m_hwInitialized;
}

bool DueTimer::enabled(void) const
{
    return m_isEnabled;
}

DueTimerMode DueTimer::getMode(void) const
{
    return m_mode;
}

void DueTimer::setFrequency(uint32_t hz)
{
    if (hz <= 0.0)
        return;

    uint32_t timerClock = 0;

    // Choose appropriate internal prescaler calculation based on mode
    if (m_mode == DueTimerMode::INTERRUPT)
        timerClock = SystemCoreClock / 32;
    else if (m_mode == DueTimerMode::PULSE_WIDTH_MODULATION)
        timerClock = SystemCoreClock / 2;
    else
        return;

    uint32_t rcValue = timerClock / hz;

    // Safety check to prevent dividing by zero behavior in hardware
    if (rcValue == 0)
        rcValue = 1;

    // Anti-stall mechanism
    uint32_t currentVal = _tc->TC_CHANNEL[_channel].TC_CV;
    bool triggerNeeded  = (currentVal > rcValue);

    _tc->TC_CHANNEL[_channel].TC_RC = rcValue;

    if (triggerNeeded)
    {
        _tc->TC_CHANNEL[_channel].TC_CCR = TC_CCR_SWTRG;
    }
}

void DueTimer::start()
{
    // Enable clock and issue software trigger to start counting from 0
    _tc->TC_CHANNEL[_channel].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
    m_isEnabled                      = true;
}

void DueTimer::stop()
{
    // Disable clock
    _tc->TC_CHANNEL[_channel].TC_CCR = TC_CCR_CLKDIS;
    m_isEnabled                      = false;
}

void DueTimer::attachInterrupt(void (*callback)(void *), void *context)
{
    DueTimer_Callbacks[_id] = callback;
    DueTimer_Contexts[_id]  = context;
}

void DueTimer::detachInterrupt()
{
    DueTimer_Callbacks[_id] = nullptr;
}

// -----------------------------------------------------------------------------
// BARE METAL INTERRUPT SERVICE ROUTINES (ISRs)
// -----------------------------------------------------------------------------
static inline void handle_isr(uint8_t id, Tc *tc, uint8_t channel)
{
    uint32_t status = tc->TC_CHANNEL[channel].TC_SR;
    if ((status & TC_SR_CPCS) && DueTimer_Callbacks[id])
    {
        DueTimer_Callbacks[id](DueTimer_Contexts[id]);
    }
}

extern "C" {
void TC0_Handler()
{
    handle_isr(0, TC0, 0);
}
void TC1_Handler()
{
    handle_isr(1, TC0, 1);
}
void TC2_Handler()
{
    handle_isr(2, TC0, 2);
}
void TC3_Handler()
{
    handle_isr(3, TC1, 0);
}
void TC4_Handler()
{
    handle_isr(4, TC1, 1);
}
void TC5_Handler()
{
    handle_isr(5, TC1, 2);
}
void TC6_Handler()
{
    handle_isr(6, TC2, 0);
}
void TC7_Handler()
{
    handle_isr(7, TC2, 1);
}
void TC8_Handler()
{
    handle_isr(8, TC2, 2);
}
}