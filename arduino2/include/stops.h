#ifndef STOPS_H
#define STOPS_H

#include "pins.h"

void setupArtDecoStops();
void sendAllStopARTDECO();
void sendStateStopARTDECO(byte *buff, int count);
void verificationStopsArdko();

#endif // STOPS_H