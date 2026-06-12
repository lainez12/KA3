#ifndef INSOLATION_H
#define INSOLATION_H

#include <Arduino.h>

#include "pins.h"

void setupInsolation();
void stopInsolation(char code);
void initInsolation(byte *buff, int count);
void startInsolationCycle(unsigned long time, unsigned int power, unsigned int couronne);
void moyennageInsol(void);
void multiplexFeedback();
void interruptExposure();
void loopInsolation();
unsigned long getCycleTime();
int getNumberCycles();

#endif