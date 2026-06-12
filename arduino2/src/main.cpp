/*
 *	Programme pour Arduino Due externe (Programming port ttyACM0)
 *	Revision 8p arduino2 v1.15 (18/02/2026)
 */

#include <SPI.h>
#include <Wire.h>
#include <string.h>

#include "deck.h"
#include "insolation.h"
#include "pins.h"
#include "stops.h"
#include "temperature.h"
#include "vacuum.h"

#define VERSION          "? : Arduino2 8p v1.15"
#define VERSION_STR_SIZE 21u

#define INTERRUP_SOFT 1
#define INTERRUP_POW  6
// 'E'mergency 'S'top
#define EMERGENCY_STOP_SEQ_SIZE 4
#define SWITCH_TIME_RELAIS_5V   3000 // milliseconds
#define HEARTBEAT_INTERVAL      1000 // 1 second

// Définition des prototypes du .ino
void finishExtinction(void);
void checkPow(void);
void triggerEmergencyStop(void);
void orderExtinction(void);
static void sendHeartbeat(void);

// Définition des variables globales
volatile unsigned long forcePrevTime                             = 0;
unsigned long debounce                                           = 0;
unsigned long Debounce_5V                                        = 0;
unsigned long LastStatePow                                       = HIGH;
bool statePowCheck                                               = true;
bool StatePow                                                    = true;
static const uint8_t EMERGENCY_STOP_SEQ[EMERGENCY_STOP_SEQ_SIZE] = {3u, 'E', 0x7F, 0x7F};
unsigned long lastHeartbeatTime                                  = 0;

void setup()
{
    Serial.begin(115200);

    analogWriteResolution(12);
    analogReadResolution(12);
    setupArtDecoStops();
    setupVacuumsensor();
    setupElectrovanne();
    setupDeck();
    setupInsolation();

    pinMode(POW_BUTTON, INPUT);
    pinMode(ORDER_EXTINCT, OUTPUT);
    pinMode(ORDER_EXTINCT_10V, OUTPUT);
    pinMode(ORDER_EXTINCT_3_3V, OUTPUT);
    pinMode(ORDER_EXTINCT_5V, OUTPUT);
    pinMode(EMERGENCY_STOP, INPUT);

    digitalWrite(ORDER_EXTINCT, HIGH);
    digitalWrite(ORDER_EXTINCT_10V, HIGH);
    digitalWrite(ORDER_EXTINCT_3_3V, HIGH);
    digitalWrite(ORDER_EXTINCT_5V, LOW);

    attachInterrupt(POW_BUTTON, orderExtinction, FALLING);
    attachInterrupt(EMERGENCY_STOP, triggerEmergencyStop, RISING);

    /* Ajout d'une fonction délais pour commuter à 1 une fois le relais du 5V */

    delay(SWITCH_TIME_RELAIS_5V);
    digitalWrite(ORDER_EXTINCT_5V, HIGH);
}

void loop()
{
    // sendHeartbeat();

    /*-----------------------DEBUT GESTION MOTEUR TAB-------------------------------*/
    //  Verification des butees
    VerificationStops();

    //  Limiteur de couple
    loopDeckTorque();

    /*----------------------- FIN GESTION MOTEUR TAB-------------------------------*/
    /*-----------------------DEBUT GESTION BUTÉES ARDKO-------------------------------*/

    //  Verification des butees
    verificationStopsArdko();
    /*-----------------------FIN GESTION BUTÉES ARDKO-------------------------------*/

    /*-----------------------DEBUT GESTION DU VIDE-------------------------------*/

    //  Verification de l'état
    verificationStatesVacuum();

    /*-----------------------FIN GESTION DU VIDE-------------------------------*/

    /*-----------------------DEBUT GESTION INSOL-------------------------------*/

    loopInsolation();

    /*-----------------------FIN GESTION INSOL-------------------------------*/
    // Système Anti-rebond Bouton Power
    if (StatePow)
    {
        checkPow();
    }
}

static void sendHeartbeat(void)
{
    unsigned long currentMillis = millis();

    if (currentMillis - lastHeartbeatTime >= HEARTBEAT_INTERVAL)
    {
        lastHeartbeatTime      = currentMillis;
        byte heartbeatPacket[] = {0x02, 'H', 'B'};
        Serial.write(heartbeatPacket, sizeof(heartbeatPacket));
    }
}

void serialEvent()
{
    byte buff[64] = {0};

    int nb    = int(Serial.read());
    int count = Serial.readBytes((char *)buff, nb);

    if (count == nb)
    {
        switch (buff[0])
        {
        case 'A':
        {
            if (buff[1] == 'C' || buff[1] == 'c')
            {
                toggleCompressedAirValveState(buff, count);
            }
            break;
        }
        case 'C':
        {
            moveContinuousMotor(buff, count);
            break;
        }
        case 'T':
        {
            setTorqueLimit(buff, count);
            break;
        }
        case 'V':
        {
            if (buff[1] == 'E' || buff[1] == 'e')
            {
                setSolenoid(buff, count);
            }
            break;
        }
        case 'I':
        {
            if (getCycleTime() == 0 && getNumberCycles() == 0)
            {
                initInsolation(buff, count);
            }
            break;
        }
        case 'S':
        {
            stopInsolation('E');
            break;
        }
        case 'E':
        {
            finishExtinction();
            break;
        }
        case '?':
        {
            if (count >= 2)
            {
                if (buff[1] == 'K' || buff[1] == 'k')
                {
                    sendStateStopARTDECO(buff, count);
                }
                else if ((buff[1] == 'V' || buff[1] == 'v'))
                {
                    if (buff[2] == 'C' || buff[2] == 'c')
                    {
                        sendStateSensor(buff, count);
                    }
                    else if (buff[2] == 'E' || buff[2] == 'e')
                    {
                        getSolenoidPower(buff, count);
                    }
                }
                else if ((buff[1] == 'A' || buff[1] == 'a'))
                {
                    if (buff[2] == 'C' || buff[2] == 'c')
                    {
                        sendCompressedAirValveState();
                        sendCompressedAirSensorState();
                    }
                }
                else if (buff[1] == 'C' || buff[1] == 'c')
                {
                    sendAllMotorStops();
                }
                else if ((buff[1] == 'T' || buff[1] == 't'))
                {
                    checkTemperature(buff, count);
                }
            }
            else
            {
                sendPlainPacketSize((byte *)VERSION, VERSION_STR_SIZE);
            }
            break;
        }
        default:
            break;
        }
    }
}

void sendPlainPacketSize(byte *message, int count)
{
    byte array[count + 1];
    memcpy(&array[1], message, count);
    array[0] = count;
    Serial.write(array, count + 1);
}

void finishExtinction(void)
{
    if (digitalRead(POW_BUTTON) == LOW)
    {
        delay(30000);
        digitalWrite(ORDER_EXTINCT, LOW);
        digitalWrite(ORDER_EXTINCT_10V, LOW);
        digitalWrite(ORDER_EXTINCT_3_3V, LOW);
        digitalWrite(ORDER_EXTINCT_5V, LOW);
    }
}

void orderExtinction()
{
    StatePow = true;
}

void checkPow(void)
{
    uint32_t reading = digitalRead(POW_BUTTON);

    if (reading != LastStatePow)
    {
        debounce = millis();
    }
    if (millis() - debounce >= 1000)
    {
        if (reading != statePowCheck)
        {
            if (getNumberCycles() > 0)
            {
                stopInsolation('E');
            }
            byte buff[3] = {2, 'E', 'E'};
            Serial.write(buff, 3);
            statePowCheck = reading;
            StatePow      = false;
        }
    }
    LastStatePow = reading;
}

void triggerEmergencyStop(void)
{
    Serial.write(EMERGENCY_STOP_SEQ, EMERGENCY_STOP_SEQ_SIZE);
    interruptExposure(); // Legacy behaviour
}
