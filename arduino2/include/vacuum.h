#ifndef VACUUM_H
#define VACUUM_H

#include <Arduino.h>

#include "pins.h"

void setupVacuumsensor();
void setupElectrovanne();
void sendStateSensor(char *buff, int count);
void getSolenoidPower(char *buff, int count);
void sendCompressedAirValveState(void);
void sendCompressedAirSensorState(void);
void sendCompressedAirSensorState(bool pinState);
void toggleCompressedAirValveState(char *buff, int count);
void verificationStatesVacuum();
void setSolenoid(char *buff, int count);

#endif // VACUUM_H