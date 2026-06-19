#include "DueTimerPWM.hpp"

DueTimerPWM::DueTimerPWM(uint8_t arduinoPin) : 
    _pin(arduinoPin), 
    _tcChannel(nullptr)
{
}

bool DueTimerPWM::begin(uint32_t frequency, uint32_t maxResolution)
{
    _maxResolution = maxResolution;

    uint32_t pmc_id;
    Tc* tc_block;
    uint32_t channel_id;
    
    if (_pin == 12) {
      
        pmc_enable_periph_clk(ID_PIOD);
        PIO_Configure(PIOD, PIO_PERIPH_B, PIO_PD8, PIO_DEFAULT);
        
        pmc_id = ID_TC8;
        tc_block = TC2;
        channel_id = 2;
        _isTIOA = false;
    }
    else {
        return false;
    }

    _tcChannel = &(tc_block->TC_CHANNEL[channel_id]);

    pmc_enable_periph_clk(pmc_id);

    _tcChannel->TC_CCR = TC_CCR_CLKDIS;

    uint32_t cmr = TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;

    if (_isTIOA) {
        cmr |= TC_CMR_ACPC_SET | TC_CMR_ACPA_CLEAR;
    } else {
        cmr |= TC_CMR_BCPC_SET | TC_CMR_BCPB_CLEAR;
    }

    _tcChannel->TC_CMR = cmr;

    uint32_t rc = (SystemCoreClock / 2) / frequency;
    _tcChannel->TC_RC = rc;
    
    if (_isTIOA) _tcChannel->TC_RA = 0;
    else         _tcChannel->TC_RB = 0;

    _tcChannel->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;

    return true;
}

void DueTimerPWM::setDuty(uint32_t dutyValue)
{
    if (!_tcChannel) return;

    if (dutyValue > _maxResolution) dutyValue = _maxResolution;

    uint32_t rc = _tcChannel->TC_RC;
    
    uint32_t duty_register_value = (rc * dutyValue) / _maxResolution;

    if (duty_register_value == 0) duty_register_value = 1; 
    if (dutyValue == 0) duty_register_value = rc + 1; 

    if (_isTIOA) {
        _tcChannel->TC_RA = duty_register_value;
    } else {
        _tcChannel->TC_RB = duty_register_value;
    }
}

void DueTimerPWM::stop()
{
    if (!_tcChannel) return;
    this->setDuty(0);
}