#ifndef VACUUM_H
#define VACUUM_H

#include <Arduino.h>

#include "pins.h"

void setupVacuumsensor();
void setupElectrovanne();
void sendStateSensor(byte *buff, int count);
void getSolenoidPower(byte *buff, int count);
void sendCompressedAirValveState(void);
void sendCompressedAirSensorState(void);
void sendCompressedAirSensorState(bool pinState);
void toggleCompressedAirValveState(byte *buff, int count);
void verificationStatesVacuum();
void setSolenoid(byte *buff, int count);

#endif // VACUUM_H