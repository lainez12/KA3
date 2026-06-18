#ifndef KUTILS_H
#define KUTILS_H

#include <Arduino.h>

// Helper function: Converts an integer to an ASCII string in a pre-allocated buffer
// Returns the number of characters written.
uint16_t intToAscii(uint8_t *buffer, uint32_t value);

#endif // KUTILS_H
