/*
 *	Programme pour Arduino Due externe (Programming port ttyACM0)
 *	Revision 8p arduino2 v1.15 (18/02/2026)
 */

#include <SPI.h>
#include <Wire.h>
#include <string.h>

#include "SerialTXHandler.h"
#include "deck.h"
#include "definitions.h"
#include "insolation.h"
#include "pins.h"
#include "stops.h"
#include "temperature.h"
#include "vacuum.h"

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
static const uint8_t EMERGENCY_STOP_SEQ[EMERGENCY_STOP_SEQ_SIZE] = {'E', 0x7F, 0x7F};
unsigned long lastHeartbeatTime                                  = 0;

void processInstruction(char *buff, uint32_t size);

void setup()
{
    Serial.begin(115200);
    Com::send(serial_packet_t((const uint8_t *)VERSION, VERSION_STR_SIZE));

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

    Com::processPackets();
}

static void sendHeartbeat(void)
{
    unsigned long currentMillis = millis();

    if (currentMillis - lastHeartbeatTime >= HEARTBEAT_INTERVAL)
    {
        lastHeartbeatTime               = currentMillis;
        const uint8_t heartbeatPacket[] = {'H', 'B'};
        Com::send(serial_packet_t(heartbeatPacket, sizeof(heartbeatPacket)));
    }
}

// ---------------------------------------------------------------------
// Communication Core
// ---------------------------------------------------------------------

void serialEvent()
{
    // Static state machine variables to parse incoming UART packets asynchronously.
    // This entirely replaces blocking calls like Serial.readBytes(), ensuring the
    // main loop (and polling tasks) never stall while waiting for a packet to finish arriving.
    static bool rxReading        = false; // false: Waiting for Length, true: Reading Data
    static uint8_t rxExpectedLen = 0;     // Number of bytes to read
    static int rxIndex           = 0;     // Current buffer position
    static char rxBuffer[MAXIMUM_SERIAL_INSTRUCTION_SIZE];

    while (Serial.available() > 0)
    {
        byte incoming = Serial.read();

        if (!rxReading)
        {
            rxExpectedLen = incoming;
            rxIndex       = 0;

            if (rxExpectedLen > 0 && rxExpectedLen < MAXIMUM_SERIAL_INSTRUCTION_SIZE)
                rxReading = true;
        }
        else
        {
            rxBuffer[rxIndex++] = (char)incoming;

            if (rxIndex >= rxExpectedLen)
            {
                processInstruction(rxBuffer, rxExpectedLen);
                rxReading = false;
            }
        }
    }
}

void processInstruction(char *buff, uint32_t size)
{
    if (size == 0)
        return;

    switch (buff[0])
    {
    case 'A':
    {
        if (size > 1 && (buff[1] == 'C' || buff[1] == 'c'))
        {
            toggleCompressedAirValveState(buff, size);
        }
        break;
    }
    case 'C':
    {
        moveContinuousMotor(buff, size);
        break;
    }
    case 'T':
    {
        setTorqueLimit(buff, size);
        break;
    }
    case 'V':
    {
        if (size > 1 && (buff[1] == 'E' || buff[1] == 'e'))
        {
            setSolenoid(buff, size);
        }
        break;
    }
    case 'I':
    {
        if (getCycleTime() == 0 && getNumberCycles() == 0)
        {
            initInsolation(buff, size);
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
        if (size >= 2)
        {
            if (buff[1] == 'K' || buff[1] == 'k')
            {
                sendStateStopARTDECO(buff, size);
            }
            else if ((buff[1] == 'V' || buff[1] == 'v'))
            {
                if (size >= 3 && (buff[2] == 'C' || buff[2] == 'c'))
                {
                    sendStateSensor(buff, size);
                }
                else if (size >= 3 && (buff[2] == 'E' || buff[2] == 'e'))
                {
                    getSolenoidPower(buff, size);
                }
            }
            else if ((buff[1] == 'A' || buff[1] == 'a'))
            {
                if (size >= 3 && (buff[2] == 'C' || buff[2] == 'c'))
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
                checkTemperature(buff, size);
            }
        }
        else
        {
            Com::send(serial_packet_t((const uint8_t *)VERSION, VERSION_STR_SIZE));
        }
        break;
    }
    default:
        break;
    }
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
            const uint8_t buff[] = {'E', 'E'};
            Com::send(serial_packet_t(buff, sizeof(buff)));
            statePowCheck = reading;
            StatePow      = false;
        }
    }
    LastStatePow = reading;
}

void triggerEmergencyStop(void)
{
    Com::send(serial_packet_t(EMERGENCY_STOP_SEQ, EMERGENCY_STOP_SEQ_SIZE));
    interruptExposure(); // Legacy behaviour
}
