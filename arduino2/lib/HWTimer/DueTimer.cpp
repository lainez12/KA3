#include "DueTimer.hpp"

// Global array to store the callbacks for all 9 timers
static void (*DueTimer_Callbacks[9])(void *) = {nullptr};
// Global array to store the callbacks contexts (passed as argument to the function)
static void *DueTimer_Contexts[9] = {nullptr};
// Resource management array to prevent multiple objects from hijacking the same timer
static DueTimer *DueTimer_Allocations[9] = {nullptr};

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

    this->attachInterrupt(callback, context);
    this->_initHardware();
    this->setFrequency(hz);

    return true;
}

void DueTimer::freeTimer()
{
    if (DueTimer_Allocations[_id] == this)
    {
        this->stop();
        this->detachInterrupt();
        NVIC_DisableIRQ(_irq);

        // Disable Peripheral Clock to save power
        if (_pmcId < 32)
            PMC->PMC_PCDR0 = (1 << _pmcId);
        else
            PMC->PMC_PCDR1 = (1 << (_pmcId - 32));

        DueTimer_Allocations[_id] = nullptr;
        m_hwInitialized           = false;
    }
}

void DueTimer::_initHardware()
{
    // 1. Enable Peripheral Clock for this specific Timer Channel
    if (_pmcId < 32)
        PMC->PMC_PCER0 = (1 << _pmcId);
    else
        PMC->PMC_PCER1 = (1 << (_pmcId - 32));
    // 2. Disable timer clock before configuring
    _tc->TC_CHANNEL[_channel].TC_CCR = TC_CCR_CLKDIS;
    // 3. Disable all interrupts for this timer channel while configuring
    _tc->TC_CHANNEL[_channel].TC_IDR = 0xFFFFFFFF;
    // 4. Read Status Register to clear any pending flags
    _tc->TC_CHANNEL[_channel].TC_SR;
    // 5. Configure Channel Mode Register
    // WAVE = Waveform mode
    // WAVSEL_UP_RC = Up counter with automatic trigger (reset) on RC compare
    // TCCLKS_TIMER_CLOCK3 = Master Clock / 32
    _tc->TC_CHANNEL[_channel].TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK3;
    // 6. Enable RC Compare Interrupt (CPCS)
    _tc->TC_CHANNEL[_channel].TC_IER = TC_IER_CPCS;
    // 7. Configure and Enable timer interrupt in the NVIC
    NVIC_ClearPendingIRQ(_irq);
    NVIC_SetPriority(_irq, 0); // Give highest default priority to timers
    NVIC_EnableIRQ(_irq);
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

void DueTimer::setFrequency(uint32_t hz)
{
    if (hz <= 0.0)
        return;

    // Division per 32 as we used `TC_CMR_TCCLKS_TIMER_CLOCK3` to configure the clock
    uint32_t timerClock = SystemCoreClock / 32;
    uint32_t rcValue    = timerClock / hz;

    // Safety check to prevent dividing by zero behavior in hardware
    if (rcValue == 0)
        rcValue = 1;

    // Read current counter value (CV)
    uint32_t currentVal = _tc->TC_CHANNEL[_channel].TC_CV;
    // Anti-stall mechanism:
    // If the timer is already past the new target, it will count all the way
    // to 0xFFFFFFFF before wrapping around, causing a huge delay.
    // We force a software trigger to reset it instantly.
    bool triggerNeeded = (currentVal > rcValue);

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
    // Set callback for this timer
    DueTimer_Callbacks[_id] = callback;
    DueTimer_Contexts[_id]  = context;
}

void DueTimer::detachInterrupt()
{
    // Reset callback for this timer
    DueTimer_Callbacks[_id] = nullptr;
}

// -----------------------------------------------------------------------------
// BARE METAL INTERRUPT SERVICE ROUTINES (ISRs)
// -----------------------------------------------------------------------------
// These 9 functions are automatically called by the ARM Cortex-M3 core
// when the respective timer fires.

static inline void handle_isr(uint8_t id, Tc *tc, uint8_t channel)
{
    // Reading TC_SR clears the interrupt flag so it doesn't fire continuously
    uint32_t status = tc->TC_CHANNEL[channel].TC_SR;

    // If it was an RC compare match, fire the user callback
    if ((status & TC_SR_CPCS) && DueTimer_Callbacks[id])
    {
        DueTimer_Callbacks[id](DueTimer_Contexts[id]);
    }
}

extern "C"
{
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
