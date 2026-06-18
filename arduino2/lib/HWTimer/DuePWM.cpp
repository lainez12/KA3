#include "DuePWM.hpp"
#include "DuePWMPins.hpp"

DuePWM::DuePWM(uint8_t arduinoPin)
{
    _pin     = arduinoPin;
    _channel = 255; // Invalid by default
}

bool DuePWM::begin(uint32_t frequency, uint32_t maxResolution)
{
    _resolution = maxResolution;

    // Mapping the Arduino Pin to the PWM and Peripheral Channels of the SAM3X8E
    const HW_PWM_PinInfo* pinData = nullptr;

    for (uint8_t i = 0; i < NUM_HW_PWM_PINS; i++)
    {
        if (due_hw_pwm_pins[i].arduinoPin == _pin)
        {
            pinData = &due_hw_pwm_pins[i];
            break;
        }
    }

    if (pinData == nullptr)
    {
        return false;
    }

    _channel = pinData->channel;

    // Turn on the PWM controller's clock (The PWM ID is 36)
    // Since 36 is greater than 31, we use PCER1 (Peripheral Clock Enable Register 1)
    PMC->PMC_PCER1 |= (1 << (ID_PWM - 32));

    // Disconnect the pin from the normal control (GPIO) and assign it to the PWM peripheral
    pinData->pioPort->PIO_PDR = pinData->pinMask; // Disable Register

    // Assign to the correct peripheral (A or B; on the Due, almost all PWMs are MUX B)
    // pioPort->PIO_ABSR |= peripheral;
    pinData->pioPort->PIO_ABSR |= pinData->pinMask;

    // Configure the PWM master clock (Clock A)
    // We use the master clock (84 MHz) divided by a prescaler.
    uint32_t clk_div  = 1;
    uint32_t clk_freq = SystemCoreClock / clk_div;

    // We configure Clock A to provide the “ticks” needed for our frequency and resolution
    // Formula: Clock_Frequency_A = Desired_Frequency * Maximum_Resolution
    uint32_t clockA_divider = clk_freq / (frequency * _resolution);

    PWM->PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(clockA_divider);

    // Configure the specific Channel
    // Use Clock A, align left, normal polarity
    PWM->PWM_CH_NUM[_channel].PWM_CMR = PWM_CMR_CPRE_CLKA;

    // Configure the Period (maximum resolution)
    PWM->PWM_CH_NUM[_channel].PWM_CPRD = _resolution;

    // Start with Duty Cycle set to 0 (Motor Off)
    PWM->PWM_CH_NUM[_channel].PWM_CDTY = 0;

    // Enable the PWM channel
    PWM->PWM_ENA = (1 << _channel);

    return true;
}

void DuePWM::setDuty(uint32_t dutyValue)
{
    if (_channel == 255)
        return; // Not initialized

    // Limit the duty to the maximum allowed value
    if (dutyValue > _resolution)
        dutyValue = _resolution;

    // Write directly to the hardware register (Instant and clean update)
    // Update Register (CPRDUPD) is used to change the duty safely in the next cycle
    PWM->PWM_CH_NUM[_channel].PWM_CDTYUPD = dutyValue;
}

void DuePWM::stop()
{
    if (_channel == 255)
        return;

    // Disable the channel
    PWM->PWM_DIS = (1 << _channel);
    // Force duty to 0
    PWM->PWM_CH_NUM[_channel].PWM_CDTYUPD = 0;
}