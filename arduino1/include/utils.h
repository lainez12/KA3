#ifndef KLOE_UTILS_H
#define KLOE_UTILS_H

#include <Arduino.h>

#define INVALID_MOTOR_INDEX UINT8_MAX
#define INVALID_LIMIT_INDEX UINT8_MAX

namespace KUtils
{
    uint8_t asciiByteToMotorIndex(char byte);
    char motorIndexToAsciiByte(uint8_t idx);
    uint8_t limitIndexToByteCode(uint8_t index);
    uint8_t stopByteCodeToIndex(uint8_t byteCode);
}

#endif // KLOE_UTILS_H
