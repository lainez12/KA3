#include "utils.h"

namespace KUtils
{

    uint8_t motorByteCodeToIndex(char byte)
    {
        if (byte < '1' || byte > '8' || byte == '7')
            return INVALID_MOTOR_INDEX; // error

        if (byte == '8')
            byte--; // Motor '8' is at index 6 so we decrement before
        return byte - '1';
    }

    char motorIndexToByteCode(uint8_t idx)
    {
        if (idx > 6)
            return INVALID_MOTOR_INDEX; // error

        if (idx == 6)
            idx++; // Motor index 6 is theta that is represented by '8' so we increment
        return idx + '1';
    }

    uint8_t limitIndexToByteCode(uint8_t index)
    {
        if (index > 9)
            return INVALID_LIMIT_INDEX; // error

        if (index == 9)
            index += 2; // Offset 2 if index is 9
        else if (index >= 6)
            index += 1; // Offset 1 if index is in [6, 7, 8]
        return index;
    }

    uint8_t limitByteCodeToIndex(uint8_t byteCode)
    {
        if (byteCode == 0x06 || byteCode == 0x0a || byteCode > 0x0b)
            return INVALID_LIMIT_INDEX;

        if (byteCode == 0x0b)
            byteCode -= 2;
        else if (byteCode >= 0x07)
            byteCode -= 1;
        return byteCode;
    }

}
