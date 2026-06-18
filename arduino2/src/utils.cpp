#include "utils.h"

// Helper function: Converts an integer to an ASCII string in a pre-allocated buffer
// Returns the number of characters written.
uint16_t intToAscii(uint8_t *buffer, long value)
{
    uint16_t len = 0;

    // Handle 0 explicitly
    if (value == 0)
    {
        buffer[len++] = '0';
        return len;
    }

    // Handle negative numbers
    if (value < 0)
    {
        buffer[len++] = '-';
        value         = -value;
    }

    uint16_t startIdx = len;

    // Extract digits in reverse order
    while (value > 0)
    {
        buffer[len++] = '0' + (value % 10);
        value /= 10;
    }

    // Reverse the digits in-place to get the correct order
    uint16_t endIdx = len - 1;
    while (startIdx < endIdx)
    {
        uint8_t temp     = buffer[startIdx];
        buffer[startIdx] = buffer[endIdx];
        buffer[endIdx]   = temp;
        startIdx++;
        endIdx--;
    }

    return len;
}
