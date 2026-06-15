// @note: https://en.wikipedia.org/wiki/Moving_average

#ifndef EMA_FILTER_HPP_
#define EMA_FILTER_HPP_

#include <Arduino.h>

// --- CONFIGURATION ---
// SCALING FACTOR: 4096 (2^12).
// This gives us ~3 decimal places of precision without using floats.
#define SCALE_FACTOR_SHIFT 12
#define FIXED_SCALE        4096
// ALPHA VALUES (Scaled)
// #define ALPHA_MIN 102 // 0.10 * 4096 = 102
#define ALPHA_MIN 2 // ~0.00025
// #define ALPHA_MAX 921 // 0.90 * 4096 = 921
#define ALPHA_MAX 204
// How many Raw ADC units of change triggers the fastest reaction?

class EMAFilter
{
public:
    EMAFilter(uint32_t (*readSensor)(), uint32_t sensitivity);
    ~EMAFilter() = default; // = delete ?

public:
    uint32_t measure(void);
    uint32_t getCurrentValue(void) const;
    void reset(void);
    inline void setSensitivity(uint32_t value)
    {
        m_sensitivity = value;
    }

private:
    uint32_t (*m_readSensor)() = nullptr;
    bool m_firstRun            = true;
    uint32_t m_currentValueScaled;
    uint32_t m_sensitivity;
};

#endif // EMA_FILTER_HPP_
