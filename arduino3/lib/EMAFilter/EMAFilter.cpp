#include "EMAFilter.hpp"

EMAFilter::EMAFilter(uint32_t (*readSensor)(), uint32_t sensitivity) :
    m_readSensor(readSensor),
    m_sensitivity(sensitivity)
{
}

// Config parameters are set to match a function call pace of 100Hz
uint32_t EMAFilter::measure(void)
{
    if (!m_readSensor)
        return (-1);

    // Read Sensor
    int32_t raw = m_readSensor(); // Use signed int for calculations

    // 1. INIT: If this is the first run, snap immediately to the value
    // Optimization to prevent the filter from taking time to climb from 0)
    if (m_firstRun)
    {
        m_currentValueScaled = raw << SCALE_FACTOR_SHIFT;
        m_firstRun           = false;
        return raw;
    }

    // 2. Get current real value (descaled)
    int32_t current_val = m_currentValueScaled >> SCALE_FACTOR_SHIFT;

    // 3. Calculate Error (Absolute difference)
    int32_t diff      = raw - current_val;
    uint32_t abs_diff = (diff < 0) ? -diff : diff;

    // 4. Calculate Adaptive Alpha (Integer Math)
    // Formula: Alpha = Min + (Boost_Factor)
    // We map the error to the range between MIN and MAX.
    int32_t current_alpha = 0;

    if (abs_diff >= m_sensitivity)
        current_alpha = ALPHA_MAX; // If error higher than sensitivity, give max importance to measured value
    else
    {
        // Linearly interpolate. Math: Boost = (Error * (Max - Min)) / Sensitivity
        int32_t range = ALPHA_MAX - ALPHA_MIN;
        int32_t boost = (abs_diff * range) / m_sensitivity;
        current_alpha = ALPHA_MIN + boost;
    }

    /*
     * --- 24-BIT ADC OVERFLOW WARNING ---
     * This next line multiplies `diff` by `current_alpha` (max 1024).
     * An int32_t will overflow if this multiplication exceeds 2,147,483,647.
     * Therefore, if `diff` > 2,097,151, IT WILL OVERFLOW AND FAIL.
     *
     * - 10-bit, 12-bit, 16-bit ADCs: 100% SAFE (Max diff is 65,535).
     * - 24-bit ADCs (e.g., HX711): DANGER.
     *
     * If using a 24-bit ADC where sudden impacts exceed 2,000,000 counts,
     * you MUST cast to 64-bit math for this single step like so:
     *
     * int64_t diff64 = (int64_t)diff;
     * m_currentValueScaled += (int32_t)(diff64 * current_alpha);
     */

    // 5. Apply EMA Formula (Fixed Point Optimized)
    // Standard Formula: New = Old + Alpha * (Raw - Old)
    // Note: 'diff' is (Raw - Old). 'current_alpha' is scaled by 1024.
    // So (diff * current_alpha) produces a result scaled by 1024.
    // This matches our stored scaled-factor value format (`m_currentValueScaled`).
    m_currentValueScaled += (diff * current_alpha);

    // 6. Return final value (Descaled)
    return (m_currentValueScaled >> SCALE_FACTOR_SHIFT);
}

uint32_t EMAFilter::getCurrentValue(void) const
{
    return (m_currentValueScaled >> SCALE_FACTOR_SHIFT);
}

void EMAFilter::reset(void)
{
    m_currentValueScaled = 0;
    m_firstRun           = true;
}
