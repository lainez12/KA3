#include "utils.h"

uint8_t asciiByteToMotorIndex(char byte)
{
    if (byte < '1' || byte > '8' || byte == '7')
        return INVALID_MOTOR_INDEX; // error

    if (byte == '8')
        byte--; // Motor '8' is at index 6 so we decrement before
    return byte - '1';
}

char motorIndexToAsciiByte(uint8_t idx)
{
    if (idx > 6)
        return INVALID_MOTOR_INDEX; // error

    if (idx == 6)
        idx++; // Motor index 6 is theta that is represented by '8' so we increment
    return idx + '1';
}

uint8_t stopIndexToByteCode(uint8_t index)
{
    if (index > 9)
        return INVALID_STOP_INDEX; // error

    if (index == 9)
        index += 2; // Offset 2 if index is 9
    else if (index >= 6)
        index += 1; // Offset 1 if index is in [6, 7, 8]
    return index;
}

uint8_t stopByteCodeToIndex(uint8_t byteCode)
{
    if (byteCode == 0x06 || byteCode == 0x0a || byteCode > 0x0b)
        return INVALID_STOP_INDEX;

    if (byteCode == 0x0b)
        byteCode -= 2;
    else if (byteCode >= 0x07)
        byteCode -= 1;
    return byteCode;
}
