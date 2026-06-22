/*
 *  Arduino 2 (Kub3.8i) firmware
 */

#include <Arduino.h>

#include "SerialTXHandler.h"
#include "ardko.h"
#include "deck.h"
#include "definitions.h"
#include "insolation.h"
#include "pins.h"
#include "temperature.h"
#include "vacuum.h"

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------
#define EMERGENCY_STOP_SEQ_SIZE 4
#define SWITCH_TIME_RELAIS_5V   3000 // milliseconds
#define HEARTBEAT_INTERVAL      1000 // 1 second

// -----------------------------------------------------------------------------
// Global States
// -----------------------------------------------------------------------------
static const uint8_t EMERGENCY_STOP_SEQ[EMERGENCY_STOP_SEQ_SIZE] = {'E', 0x7F, 0x7F};

unsigned long debounce          = 0;
unsigned long lastStatePow      = HIGH;
bool statePowCheck              = true;
bool statePow                   = true;
unsigned long lastHeartbeatTime = 0;

// -----------------------------------------------------------------------------
// Forward Declarations
// -----------------------------------------------------------------------------
void finishExtinction(void);
void checkPow(void);
void triggerEmergencyStop(void);
void orderExtinction(void);
// static void sendHeartbeat(void);
void processInstruction(char *buff, uint32_t size);

// -----------------------------------------------------------------------------
// Main Arduino Setup
// -----------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);

    // Announce identity natively using TX handler
    Com::send(serial_packet_t((const uint8_t *)VERSION, VERSION_SIZE));

    analogWriteResolution(12);
    analogReadResolution(12);

    // Subsystems Setup
    Ardko::setup();
    Vacuum::setup();
    Deck::setup();
    Insolation::setup();

    // Hardware Power Routing
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

    attachInterrupt(digitalPinToInterrupt(POW_BUTTON), orderExtinction, FALLING);
    attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP), triggerEmergencyStop, RISING);

    // Delay lock execution to stabilize 5V relay line
    delay(SWITCH_TIME_RELAIS_5V);
    digitalWrite(ORDER_EXTINCT_5V, HIGH);
}

// -----------------------------------------------------------------------------
// Main Execution Loop
// -----------------------------------------------------------------------------
void loop()
{
    // Execution loops for subsystem polling safeties
    Deck::loop();
    Ardko::loop();
    Vacuum::loop();
    Insolation::loop();

    // Power Debounce logic
    if (statePow)
    {
        checkPow();
    }

    Com::processPackets();
}

// static void sendHeartbeat(void)
// {
//     unsigned long currentMillis = millis();
//     if (currentMillis - lastHeartbeatTime >= HEARTBEAT_INTERVAL)
//     {
//         lastHeartbeatTime               = currentMillis;
//         const uint8_t heartbeatPacket[] = {'H', 'B'};
//         Com::send(serial_packet_t(heartbeatPacket, sizeof(heartbeatPacket)));
//     }
// }

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
        if (size > 1 && (buff[1] == 'C' || buff[1] == 'c'))
            Vacuum::toggleCompressedAirValveState(buff, size);
        break;

    case 'C':
    {
        if (size > 1 && (buff[1] == 'F' || buff[1] == 'f'))
            Deck::startMotor(buff, size);
        break;
    }

    case 'T':
        Deck::setTorqueLimit(buff, size);
        break;

    case 'V':
        if (size > 1 && (buff[1] == 'E' || buff[1] == 'e'))
            Vacuum::setSolenoid(buff, size);
        break;

    case 'I':
        if (Insolation::getCycleTime() == 0 && Insolation::getNumberCycles() == 0)
            Insolation::startCycle(buff, size);
        break;

    case 'S':
        Insolation::stopCycle('E');
        break;

    case 'E':
        finishExtinction();
        break;

    case '?':
        if (size >= 2)
        {
            if (buff[1] == 'K' || buff[1] == 'k')
            {
                Ardko::sendLimitValue(buff, size);
            }
            else if ((buff[1] == 'V' || buff[1] == 'v'))
            {
                if (size >= 3 && (buff[2] == 'C' || buff[2] == 'c'))
                    Vacuum::sendStateSensor(buff, size);
                else if (size >= 3 && (buff[2] == 'E' || buff[2] == 'e'))
                    Vacuum::getSolenoidPower(buff, size);
            }
            else if ((buff[1] == 'A' || buff[1] == 'a'))
            {
                if (size >= 3 && (buff[2] == 'C' || buff[2] == 'c'))
                {
                    Vacuum::sendCompressedAirValveState();
                    Vacuum::sendCompressedAirSensorState();
                }
            }
            else if (buff[1] == 'C' || buff[1] == 'c')
            {
                Deck::sendAllLimitsValues();
            }
            else if ((buff[1] == 'T' || buff[1] == 't'))
            {
                Temperature::checkSensors(buff, size);
            }
        }
        else
        {
            Com::send(serial_packet_t((const uint8_t *)VERSION, VERSION_SIZE));
        }
        break;

    default:
        break;
    }
}

// ---------------------------------------------------------------------
// Power Safety Logic
// ---------------------------------------------------------------------

void finishExtinction(void)
{
    if (digitalRead(POW_BUTTON) == LOW)
    {
        delay(30000); // Wait strictly 30 seconds before hardware cut
        digitalWrite(ORDER_EXTINCT, LOW);
        digitalWrite(ORDER_EXTINCT_10V, LOW);
        digitalWrite(ORDER_EXTINCT_3_3V, LOW);
        digitalWrite(ORDER_EXTINCT_5V, LOW);
    }
}

void orderExtinction()
{
    statePow = true;
}

void checkPow(void)
{
    uint32_t reading = digitalRead(POW_BUTTON);

    if (reading != lastStatePow)
        debounce = millis();

    if (millis() - debounce >= 1000) // 1 second debounce
    {
        if (reading != statePowCheck)
        {
            if (Insolation::getNumberCycles() > 0)
                Insolation::stopCycle('E');

            const uint8_t buff[] = {'E', 'E'};
            Com::send(serial_packet_t(buff, sizeof(buff)));

            statePowCheck = reading;
            statePow      = false;
        }
    }
    lastStatePow = reading;
}

void triggerEmergencyStop(void)
{
    Com::send(serial_packet_t(EMERGENCY_STOP_SEQ, EMERGENCY_STOP_SEQ_SIZE));
    Insolation::interruptExposure();
}
