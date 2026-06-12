#include <Arduino.h>

#include "deck.h"

static unsigned int courseDir          = 0;
static unsigned long prevCoupleTimes   = 0;
static unsigned long previousStopsTime = 0;
static unsigned int stopStates[2]      = {HIGH, HIGH};
static int torqueLimit[2]              = {2750, 2481};

void setupDeck()
{
    pinMode(DECK_DIRECTION, OUTPUT);
    pinMode(DECK_DISABLE, OUTPUT);
    pinMode(DECK_CLOCK, OUTPUT);
    pinMode(DECK_INSOLSTOP, INPUT);
    pinMode(DECK_ALIGNSTOP, INPUT);

    digitalWrite(DECK_DISABLE, HIGH);
    digitalWrite(DECK_DIRECTION, HIGH);

    stopStates[0] = !digitalRead(DECK_ALIGNSTOP);
    stopStates[1] = !digitalRead(DECK_INSOLSTOP);
}

void loopDeckTorque()
{
    // Limiteur de couple
    if (prevCoupleTimes > 0 && (unsigned long)(millis() - prevCoupleTimes) >= 200)
    {
        prevCoupleTimes = (millis() == 0 ? 1 : millis());
        int dir;
        if (DECK_DIRECTION == 1)
            dir = 1;
        else
            dir = 0;
        if (analogRead(DECK_COUPLE) > torqueLimit[dir])
        {
            coupleStop();
        }
    }
}

void coupleStop()
{
    stopMotor();
    byte buff[4] = {3, 'C', 'L', '1'};
    Serial.write(buff, 4);
}

//  -------------------------------------------------------------------------
//                      CC Motors
//  -------------------------------------------------------------------------

/*void moveContinuousMotor(byte* buff, int count)
{
  if (count >= 3)
  {
  int partIndex;
  if (buff[1] == 'D' || buff[1] == 'd')
    partIndex = 0;
  else
    partIndex = 1;

  if (buff[2] == 'S' || buff[2] == 's')
  {
    stopMotor(partIndex);
  }
  else if(drawerEnabled || partIndex == 1)
  {
    boolean dir;
    if (buff[2] == 'F' || buff[2] == 'f')
    dir = LOW;
    else
    dir = HIGH;

    if ((dir == LOW && digitalRead(forwardStops[partIndex]) == LOW) || (dir == HIGH && digitalRead(backwardStops[partIndex]) == LOW))
    {
    digitalWrite(directions[partIndex], dir);
    courseDir[partIndex] = dir;

    int index = 3;
    int value = 0;
    while (index < count)
      {
      value = (value * 10) + (buff[index] - '0');
      index++;
    }
    if (value > 4095)
      value = 4095;
    if (value < 0)
      value = 0;

    prevCoupleTimes[partIndex] = (millis() == 0 ? 1 : millis());

    digitalWrite(disables[partIndex], LOW);
    analogWrite(clocks[partIndex], value);
    }
  }
  }
}*/

void moveContinuousMotor(byte *buff, int count)
{
    if (count >= 3)
    {
        if (buff[1] == 'F' || buff[1] == 'f')
        {
            if (buff[2] == 'S' || buff[2] == 's')
            {
                stopMotor();
            }
            else if ((buff[2] == 'B' || buff[2] == 'b') || (buff[2] == 'F' || buff[2] == 'f'))
            {
                boolean dir = ((buff[2] == 'F' || buff[2] == 'f') ? LOW : HIGH);

                if ((dir == LOW && digitalRead(DECK_ALIGNSTOP) == HIGH) || (dir == HIGH && digitalRead(DECK_INSOLSTOP) == HIGH))
                {
                    digitalWrite(DECK_DIRECTION, dir);
                    courseDir = dir;

                    int index = 3;
                    int value = 0;
                    while (index < count)
                    {
                        value = (value * 10) + (buff[index] - '0');
                        index++;
                    }
                    if (value > 4095)
                        value = 4095;
                    if (value < 0)
                        value = 0;

                    uint32_t timestamp = millis();

                    prevCoupleTimes = (timestamp == 0 ? 1 : timestamp);
                    digitalWrite(DECK_DISABLE, LOW);
                    analogWrite(DECK_CLOCK, value);
                }
            }
        }
    }
}

void stopMotor()
{
    analogWrite(DECK_CLOCK, 0);
    digitalWrite(DECK_DISABLE, HIGH);
    prevCoupleTimes = 0;
}

/*
void toggleDrawer()
{
  byte buff[3] = {2, 'C', 'T'};
  Serial.write(buff, 3);
}
*/

void stopForward()
{
    boolean state = !digitalRead(DECK_ALIGNSTOP);

    if (courseDir == LOW && state == HIGH)
    {
        stopMotor();
    }
    if (state != stopStates[0])
    {
        byte buff[5] = {4, 'C', '1', 'F', state + '0'};
        Serial.write(buff, 5);
        stopStates[0] = state;
    }
}

void stopBackward()
{
    boolean state = !digitalRead(DECK_INSOLSTOP);

    if (courseDir == HIGH && state == HIGH)
    {
        stopMotor();
    }
    if (state != stopStates[1])
    {
        byte buff[5] = {4, 'C', '1', 'B', state + '0'};
        Serial.write(buff, 5);
        stopStates[1] = state;
    }
}

void VerificationStops()
{

    if ((unsigned long)(millis() - previousStopsTime) >= 100)
    {
        previousStopsTime = millis();
        stopBackward();
        stopForward();
    }
}

void sendAllMotorStops()
{
    byte buff[5] = {4, 'C', '1', 'F', stopStates[0] + '0'};
    Serial.write(buff, 5);
    buff[3] = 'B';
    buff[4] = stopStates[1] + '0';
    Serial.write(buff, 5);
}

void setTorqueLimit(byte *buff, int count)
{
    if (count >= 4)
    {
        if (buff[1] == 'F' || buff[1] == 'f')
        {
            boolean dir;
            if (buff[2] == 'F' || buff[2] == 'f')
            {
                dir = 0;
            }
            else
            {
                dir = 1;
            }
            int i(3);
            int value(0);
            while (i < count)
            {
                value = (value * 10) + (buff[i] - '0');
                i++;
            }
            if (value >= 0 && value <= 4095)
            {
                torqueLimit[dir] = value;
                byte buff[7]     = {6, count, 'T', 'F', (dir ? 'B' : 'F'), lowByte(value >> 8), lowByte(value)};
                Serial.write(buff, 7);
            }
            else
            {
                byte buff[5] = {4, '#', '1', lowByte(value >> 8), lowByte(value)};
                Serial.write(buff, 5);
            }
        }
    }
    else
    {
        byte buff[4] = {3, '#', '2', count};
        Serial.write(buff, 4);
    }
}
