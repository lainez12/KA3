#include <string.h>

#include "SerialTXHandler.h"
#include "vacuum.h"

static boolean runnings[2]     = {false, false};
static String trame            = "";
static bool compressedAirState = false;

static unsigned int directionElectrovanne[2] = {SM_DIRECTION, SW_DIRECTION};
static unsigned int disablesElectrovanne[2]  = {SM_DISABLE, SW_DISABLE};
static unsigned int clocksElectrovanne[2]    = {SM_CLOCK, SW_CLOCK};
static unsigned int powersElectrovanne[2]    = {0, 0};

static void sendPlainPacketSizeSolenoid(byte *message, int count)
{
    Com::send(serial_packet_t(message, count));
}

void setupElectrovanne()
{
    // TODO review
    pinMode(SM_DISABLE, OUTPUT);
    pinMode(SM_DIRECTION, OUTPUT);
    pinMode(SM_CLOCK, OUTPUT);

    pinMode(SW_DISABLE, OUTPUT);
    pinMode(SW_DIRECTION, OUTPUT);
    pinMode(SW_CLOCK, OUTPUT);

    pinMode(SW_COMPRESSED_AIR, OUTPUT);

    digitalWrite(SM_DISABLE, HIGH);
    digitalWrite(SM_DIRECTION, LOW);

    digitalWrite(SW_DISABLE, HIGH);
    digitalWrite(SW_DIRECTION, LOW);

    digitalWrite(SW_COMPRESSED_AIR, LOW);
}

void setSolenoid(byte *buff, int count)
{
    if (count < 5)
    {
        return;
    }
    // TODO review
    //  "VE" + <IDELECTROVANNE> (1 octét ascii, M (mask) ou W (wafer), position 2)
    int state(buff[2] == 'M' ? 0 : 1);
    // + <ETAT> (1 octét, ascii 1 (défault) ou 0 (debug), position 3)
    int dir(buff[3] == '1' ? HIGH : LOW);
    // + <PUISSANCE> (1 à 4 octéts, positions 4 à 7)
    int index = 4;
    int value = 0;
    while (index < count)
    {
        value = (value * 10) + (buff[index] - '0');
        index++;
    }
    if (value > 4095)
    {
        value = 4095;
    }
    else if (value < 0)
    {
        value = 0;
    }

    digitalWrite(directionElectrovanne[state], dir);
    digitalWrite(disablesElectrovanne[state], LOW);
    if (powersElectrovanne[state] != value)
    {
        analogWrite(clocksElectrovanne[state], value);
        powersElectrovanne[state] = value;
        // if(state)
        // {
        //     String buff = "VEM"+ String(dir) + String(value);
        //     String trame = String(buff.length()) + buff;
        //     Serial.write(trame.c_str,trame.length);
        // }
        // else
        // {
        //     String buff = "VEW"+ String(dir) + String(value);            s
        //     String trame = String(buff.length()) + buff;
        //     Serial.write(trame.c_str,trame.length);
        // }

        // string buff = "VE"+string(value)
        /*byte buff[9] = {8, 'V', 'E', (state ? 'W' : 'M'), dir + '0'};
        buff[5] = powersElectrovanne[state] >> 24;
        buff[6] = powersElectrovanne[state] >> 16;
        buff[7] = powersElectrovanne[state] >> 8;
        buff[8] = powersElectrovanne[state];

        Serial.write(buff, 9);*/
        getSolenoidPower(buff, count);
        if (value > 0)
        {
            runnings[state] = true;
        }
        else
        {
            runnings[state] = false;
        }
    }
}

void toggleCompressedAirValveState(byte *buff, int count)
{
    if (count > 3)
        return;

    const bool compressedAirRequest = (buff[2] == '1');

    if (compressedAirState != compressedAirRequest)
    {
        digitalWrite(SW_COMPRESSED_AIR, compressedAirRequest);
        compressedAirState = compressedAirRequest;
        sendCompressedAirValveState();
    }
}

void getSolenoidPower(byte *buff, int count)
{

    if (count >= 3)
    {
        if ((buff[3] == 'M' || buff[3] == 'm') && (buff[0] == '?') || (buff[2] == 'M' || buff[2] == 'm'))
        {
            String buffTemp = "VEM" + String(digitalRead(directionElectrovanne[0])) + String(powersElectrovanne[0]);
            sendPlainPacketSizeSolenoid((byte *)buffTemp.c_str(), buffTemp.length());
        }
        else
        {
            String buffTemp = "VEW" + String(digitalRead(directionElectrovanne[1])) + String(powersElectrovanne[1]);
            sendPlainPacketSizeSolenoid((byte *)buffTemp.c_str(), buffTemp.length());
        }
    }
}
