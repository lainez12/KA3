#ifndef KLOE_UTILS_H
#define KLOE_UTILS_H

#include <Arduino.h>

#define INVALID_MOTOR_INDEX UINT8_MAX
#define INVALID_LIMIT_INDEX UINT8_MAX

namespace KUtils
{
    /**
     * @brief Translates the legacy ASCII motor character ('1'-'6', '8') into a clean
     * 0-indexed integer (0-6) for internal array lookups.
     */
    uint8_t motorByteCodeToIndex(char byte);

    /**
     * @brief Translates an internal 0-indexed motor integer back into the legacy
     * ASCII character expected by the host software.
     */
    char motorIndexToByteCode(uint8_t idx);

    /**
     * @brief Applies hardcoded offsets to limit switch indices to match the
     * host software's expected protocol format.
     */
    uint8_t limitIndexToByteCode(uint8_t index);

    /**
     * @brief Reverses the host software's offset byte code back into a clean
     * 0-indexed integer for limit array lookups.
     */
    uint8_t limitByteCodeToIndex(uint8_t byteCode);
}

#endif // KLOE_UTILS_H
