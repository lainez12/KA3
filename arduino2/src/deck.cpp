#include <Arduino.h>

#include "DueDCMotor.hpp"
#include "SerialTXHandler.h"
#include "deck.h"

static DueDCMotor deckMotor(
    due_dc_motor_pins_t{
        .disable   = DECK_DISABLE,
        .direction = DECK_DIRECTION,
        .pwm       = DECK_CLOCK,
        .torque    = DECK_COUPLE});

static unsigned int courseDir          = 0;
static unsigned long prevCoupleTimes   = 0;
static unsigned long previousStopsTime = 0;
static unsigned int stopStates[2]      = {HIGH, HIGH};
static int torqueLimit[2]              = {2750, 2481};

void setupDeck()
{
    deckMotor.setup(20000, 4095);

    digitalWrite(DECK_DISABLE, HIGH);
    digitalWrite(DECK_DIRECTION, HIGH);

    stopStates[0] = !digitalRead(DECK_ALIGNSTOP);
    stopStates[1] = !digitalRead(DECK_INSOLSTOP);
}

// volatile uint16_t spd = 0;

void loopDeckTorque()
{
    // Limiteur de couple
    if (prevCoupleTimes > 0 && (unsigned long)(millis() - prevCoupleTimes) >= 200)
    {
        prevCoupleTimes = (millis() == 0 ? 1 : millis());
        int dir         = (courseDir == HIGH) ? 1 : 0;
        if (deckMotor.readTorque() > torqueLimit[dir])
        {
            coupleStop();
        }
    }

    // static uint32_t lchk = 0;

    // if (millis() - lchk > 20)
    // {
    //     lchk = millis();
    //     spd += 5;
    //     if (spd >= 4095)
    //         spd = 0;
    //     deckMotor.setSpeed(spd);
    // }
}

void coupleStop()
{
    stopMotor();
    byte buff[] = {'C', 'L', '1'};
    Com::send(serial_packet_t(buff, sizeof(buff)));
}

//  -------------------------------------------------------------------------
//                      CC Motors
//  -------------------------------------------------------------------------

void moveContinuousMotor(char *buff, int count)
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
                    deckMotor.setDirection(dir);
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
                    deckMotor.enable(true);
                    deckMotor.setSpeed(value);
                }
            }
        }
    }
}

void stopMotor()
{
    deckMotor.stop();
    prevCoupleTimes = 0;
}

void stopForward()
{
    boolean state = !digitalRead(DECK_ALIGNSTOP);

    if (courseDir == LOW && state == HIGH)
    {
        stopMotor();
    }
    if (state != stopStates[0])
    {
        byte buff[] = {'C', '1', 'F', state + '0'};
        Com::send(serial_packet_t((uint8_t *)buff, sizeof(buff)));
        stopStates[0] = state;
    }
}

void setTorqueLimit(char *buff, int count)
{
    if (count >= 3)
    {
        int dir   = (buff[1] == 'F' || buff[1] == 'f') ? 0 : 1;
        int index = 2;
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

        torqueLimit[dir] = value;
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
        byte buff[] = {'C', '1', 'B', state + '0'};
        Com::send(serial_packet_t((uint8_t *)buff, sizeof(buff)));
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
    byte buff[] = {'C', '1', 'F', stopStates[0] + '0'};
    Com::send(serial_packet_t((uint8_t *)buff, sizeof(buff)));
    buff[2] = 'B';
    buff[3] = stopStates[1] + '0';
    Com::send(serial_packet_t((uint8_t *)buff, sizeof(buff)));
}
