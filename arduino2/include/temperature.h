#ifndef TEMPERATURE_H
#define TEMERATURE_H

#include "pins.h"
#include <Arduino.h>

#define NUMBER_IT 5

void checkTemperature(byte *buff, int count);
void checkFans();
void clearArrayMoyennageTempVoltage();
void clearArrayMoyennageFanVoltage();

#endif
