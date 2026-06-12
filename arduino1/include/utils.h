#ifndef UTILS_H_ARDUINO1
#define UTILS_H_ARDUINO1

#include "Arduino.h"

#define INVALID_MOTOR_INDEX (static_cast<uint8_t>(-1))
#define INVALID_STOP_INDEX  (static_cast<uint8_t>(-1))

uint8_t asciiByteToMotorIndex(char byte);
char motorIndexToAsciiByte(uint8_t idx);
uint8_t stopIndexToByteCode(uint8_t index);
uint8_t stopByteCodeToIndex(uint8_t byteCode);

#endif
