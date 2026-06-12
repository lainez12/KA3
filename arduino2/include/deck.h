#ifndef DECK_H
#define DECK_H

#include "pins.h"

void setupDeck();
void loopDeckTorque();

void coupleStop();
void stopMotor();
void stopForward();
void stopBackward();

void stopMotor();
void stopForward();
void stopBackward();

void VerificationStops();

void sendAllMotorStops();

void moveContinuousMotor(byte *buff, int count);
void setTorqueLimit(byte *buff, int count);

void lockDeck();
void unlockDeck();

#endif // DECK_H