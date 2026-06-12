/*
 *	Programme pour Arduino1 Kub3.8i Dec (programming port ttyACM#)
 *	Revision 8p v1.11 (27/01/2026)
 */

#include "pins.h"
#include "utils.h"

#include <Arduino.h>
#include <SPI.h>

#define ANALIMIT                        310
#define VERSION                         "? : Arduino1 8p v1.11"
#define MAXIMUM_SERIAL_INSTRUCTION_SIZE 64

#define MOTOR_TARGET_MOVEMENT_TOLERANCE 3

// Causes of motors stopping
#define STOPBY_ORDER_SOFT      0
#define STOPBY_NB_STEP         1
#define STOPBY_POSITION_CODER  2
#define STOPBY_BUTEE           3
#define STOPBY_TARGET_EXCEEDED 4

#define FREQ_TARGET_MOTOR 1000

//  Tableaux de pins
int nEnPins[7]   = {lx_nEn, ly_nEn, rx_nEn, ry_nEn, px_nEn, py_nEn, pt_nEn};
int mzeroPins[7] = {lx_M0, ly_M0, rx_M0, ry_M0, px_M0, py_M0, pt_M0};
int monePins[7]  = {lx_M1, ly_M1, rx_M1, ry_M1, px_M1, py_M1, pt_M1};
int stepPins[7]  = {lx_STEP, ly_STEP, rx_STEP, ry_STEP, px_STEP, py_STEP, pt_STEP};
int dirPins[7]   = {lx_DIR, ly_DIR, rx_DIR, ry_DIR, px_DIR, py_DIR, pt_DIR};
int stopPins[10] = {Blx, Bly, Brx, Bry, Bpx, Bpy, Bpt, NLx, NLy, NLt};
int codAPins[7]  = {lx_codA, ly_codA, rx_codA, ry_codA, px_codA, py_codA, pt_codA};
int codBPins[7]  = {lx_codB, ly_codB, rx_codB, ry_codB, px_codB, py_codB, pt_codB};

//  Variables globales
volatile unsigned long prevMotorTimes[7] = {0, 0, 0, 0, 0, 0, 0};
unsigned long prevCoderTime              = 0;
volatile boolean alignmentEnabled        = true;
volatile long numberSteps[7]             = {-1, -1, -1, -1, -1, -1, -1};
volatile char directions[7]              = {0, 0, 0, 0, 0, 0, 0};
volatile char resolutions[7]             = {0, 0, 0, 0, 0, 0, 0};
volatile unsigned long halfperiods[7]    = {100000, 100000, 100000, 100000, 100000, 100000, 100000};
volatile boolean runnings[7]             = {false, false, false, false, false, false, false};
volatile boolean stepStates[7]           = {false, false, false, false, false, false, false};
boolean stopStates[10]                   = {false, false, false, false, false, false, false, false, false, false};
boolean waferstopstate                   = false;
volatile long coderCounts[7]             = {0, 0, 0, 0, 0, 0, 0};
volatile bool coderChanged[7]            = {false, false, false, false, false, false, false};
volatile long motorTarget[7]             = {0, 0, 0, 0, 0, 0, 0};
volatile unsigned long IDCounts[7]       = {0, 0, 0, 0, 0, 0, 0};
char *packID[7]                          = {"LX", "LY", "RX", "RY", "PX", "PY", "PT"};

void startMotorToTarget(int index, long target);

void setup()
{
    Serial.begin(115200);
    sendPlainPacketSize((byte *)VERSION, 21);

    SPI.begin(leftSlave);
    SPI.begin(rightSlave);
    SPI.setDataMode(leftSlave, SPI_MODE3);
    SPI.setBitOrder(leftSlave, MSBFIRST);
    SPI.setDataMode(rightSlave, SPI_MODE3);
    SPI.setBitOrder(rightSlave, MSBFIRST);
    SPI.setClockDivider(leftSlave, 21);
    SPI.setClockDivider(rightSlave, 21);

    for (int index = 0; index < 7; ++index)
    {
        pinMode(nEnPins[index], OUTPUT);
        pinMode(mzeroPins[index], OUTPUT);
        pinMode(monePins[index], OUTPUT);
        pinMode(stepPins[index], OUTPUT);
        pinMode(dirPins[index], OUTPUT);
        pinMode(stopPins[index], INPUT);

        digitalWrite(nEnPins[index], HIGH);
        digitalWrite(stepPins[index], LOW);
    }
    pinMode(NLx, INPUT);
    pinMode(NLy, INPUT);
    pinMode(NLt, INPUT);

    for (int index = 0; index < 7; ++index)
    {
        pinMode(codAPins[index], INPUT);
        pinMode(codBPins[index], INPUT);
    }

    for (int i = 0; i < 10; ++i)
        stopStates[i] = digitalRead(stopPins[i]);

    // Interruptions
    attachInterrupt(lx_codA, lx_coderFront, CHANGE);
    attachInterrupt(rx_codA, rx_coderFront, CHANGE);
    attachInterrupt(ly_codA, ly_coderFront, CHANGE);
    attachInterrupt(ry_codA, ry_coderFront, CHANGE);
    attachInterrupt(px_codA, px_coderFront, CHANGE);
    attachInterrupt(py_codA, py_coderFront, CHANGE);
    attachInterrupt(pt_codA, pt_coderFront, CHANGE);
}

void loop()
{
    if (alignmentEnabled)
    {
        for (int index = 0; index < 7; ++index)
        {
            // Condition de moteur en cours d'utilisation
            const bool motorRunning = runnings[index] && ((unsigned long)(micros() - prevMotorTimes[index]) >= halfperiods[index]) && numberSteps[index] != 0;

            if (motorRunning)
            {
                // Condition d'arrêt pour cause de butée atteinte
                const bool motorStoppedByButee = (directions[index] == LOW && digitalRead(stopPins[index])) ||
                                                 (directions[index] == HIGH && index >= 4 && analogRead(stopPins[index + 3]) > ANALIMIT);

                if (motorStoppedByButee)
                    stopMotor(index, STOPBY_BUTEE); // Arrêt par butée
                else
                { // If motor not stopped by butée, perform step state inversion to move
                    digitalWrite(stepPins[index], stepStates[index]);
                    stepStates[index]     = !stepStates[index]; // Inverse state for next iteration
                    prevMotorTimes[index] = micros();

                    if (numberSteps[index] > 0) // When requested to move for a number of steps
                    {
                        numberSteps[index] -= 1;     // Decrement number of steps to perform
                        if (numberSteps[index] == 0) // Condition d'arrêt pour nombre pas effectués
                        {
                            stopMotor(index, STOPBY_NB_STEP);
                        }
                    }
                    else if (motorTarget[index] >= 0) // When requested to move to a specific position
                    {
                        if (abs(motorTarget[index] - coderCounts[index]) <= MOTOR_TARGET_MOVEMENT_TOLERANCE) // Condition d'arrêt pour cible atteinte
                        {
                            stopMotor(index, STOPBY_POSITION_CODER);
                            motorTarget[index] = -1;
                        }
                        else if ((directions[index] == HIGH && motorTarget[index] < coderCounts[index]) ||
                                 (directions[index] == LOW && motorTarget[index] > coderCounts[index])) // Condition d'arrêt pour cible depassée
                        {
                            stopMotor(index, STOPBY_TARGET_EXCEEDED);
                        }
                    }
                }
            }
        }
    }

    if ((unsigned long)(millis() - prevCoderTime) >= 100)
    {
        for (int index = 0; index < 10; ++index)
        {
            boolean state;

            if (index > 6)
                state = analogRead(stopPins[index]) > ANALIMIT;
            else
                state = digitalRead(stopPins[index]);

            if (state != stopStates[index]) // Detect change in stop state to update
            {
                stopStates[index] = state; // Cache stop value

                const uint8_t stopByteCode = stopIndexToByteCode(index);
                if (stopByteCode == INVALID_STOP_INDEX)
                    continue;

                byte buff[4] = {3, 'S', stopByteCode, state};
                Serial.write(buff, 4);
            }
        }

        giveCoders();
        prevCoderTime = millis();
    }
}

//  ---------------------------------------------------------------------
//                    Pilotage moteurs
//  ---------------------------------------------------------------------

void stopMotor(uint8_t motorIdx, uint8_t cause)
{
    digitalWrite(nEnPins[motorIdx], HIGH);
    digitalWrite(stepPins[motorIdx], LOW);
    runnings[motorIdx]    = false;
    numberSteps[motorIdx] = -1;
    motorTarget[motorIdx] = -1;

    const uint8_t motorByteCode = motorIndexToAsciiByte(motorIdx);
    byte buff[5]                = {4, 'S', 'S', motorByteCode, cause};

    Serial.write(buff, 5);
}

void startMotor(char *buff, int count)
{
    if (count < 6) // Invalid payload (too short) => ignore request
        return;

    const uint8_t index = asciiByteToMotorIndex(buff[1]);

    if (index == INVALID_MOTOR_INDEX) // Invalid index received, ignore request
        return;

    int freq = (int(buff[4]) << 8) + int(buff[5]);

    if (freq > 0)
        halfperiods[index] = (unsigned long)(5e5 / freq);

    if (motorTarget[index] >= 0)
    {
        runnings[index]    = false;
        motorTarget[index] = -1;
    }

    if (!runnings[index]) // If motor was not running previously
    {
        digitalWrite(nEnPins[index], LOW); // Enable motor
        stepStates[index]     = LOW;       // Initialise step state
        prevMotorTimes[index] = 0;         // Reset motor clock
    }

    // Set direction
    if (buff[2] == '1')
        directions[index] = HIGH;
    else
        directions[index] = LOW;
    digitalWrite(dirPins[index], directions[index]); // Updates pin that handles motor direction

    // Set resolution if changed
    if (resolutions[index] != buff[3])
        resolutions[index] = buff[3];

    // Set step number to perform before stopping motor
    if (count >= 10)
        numberSteps[index] = 2 * ((long(buff[6]) << 24) + (long(buff[7]) << 16) + (long(buff[8]) << 8) + long(buff[9]));
    else
        numberSteps[index] = -1;

    /**
     * À chaque fois on change le mode de pin (Mzero et Mone)
     * pour le cas où nous étions auparavant en résolution 1/32 ou 1/4.
     */
    pinMode(mzeroPins[index], OUTPUT);
    pinMode(monePins[index], OUTPUT);
    if (resolutions[index] == 32)
    {
        pinMode(mzeroPins[index], INPUT);
        // digitalWrite(mzeroPins[index], HIGH);
        digitalWrite(monePins[index], HIGH);
    }
    else if (resolutions[index] == 16)
    {
        digitalWrite(mzeroPins[index], HIGH);
        digitalWrite(monePins[index], HIGH);
    }
    else if (resolutions[index] == 8)
    {
        digitalWrite(mzeroPins[index], LOW);
        digitalWrite(monePins[index], HIGH);
    }
    else if (resolutions[index] == 4)
    {
        // digitalWrite(mzeroPins[index], LOW);
        pinMode(mzeroPins[index], INPUT);
        digitalWrite(monePins[index], LOW);
    }
    else if (resolutions[index] == 2)
    {
        digitalWrite(mzeroPins[index], HIGH);
        digitalWrite(monePins[index], LOW);
    }
    else
    {
        digitalWrite(mzeroPins[index], LOW);
        digitalWrite(monePins[index], LOW);
    }

    runnings[index] = true; // Set motor as running
}

void enableMotor(char *buff, int count)
{
    if (count < 3) // Invalid request
        return;

    const uint8_t idx = asciiByteToMotorIndex(buff[1]);

    if (idx == INVALID_MOTOR_INDEX) // Ignore
        return;

    const bool enable = (bool)buff[2];

    digitalWrite(nEnPins[idx], enable ? LOW : HIGH);
}

void moveToTarget(char *buff, int count)
{
    if (count < 3) // Invalid request, ignore
        return;

    const uint8_t motorIdx = asciiByteToMotorIndex(buff[1]);
    if (motorIdx == INVALID_MOTOR_INDEX)
        return;

    long target = 0;
    int idx     = 2;
    bool neg    = false;

    if (buff[idx] == '-') // Handle negative number
    {
        neg = true;
        ++idx;
    }
    while (buff[idx] != '#' && idx < count)
        target = (target * 10) + (buff[idx++] - '0');
    if (neg)
        target *= (-1);

    startMotorToTarget(motorIdx, target);
}

void startMotorToTarget(uint8_t index, long target)
{
    motorTarget[index] = target;
    halfperiods[index] = (unsigned long)(5e5 / FREQ_TARGET_MOTOR);

    if (motorTarget[index] > coderCounts[index])
        directions[index] = HIGH;
    else
        directions[index] = LOW;

    if (index >= 0 && index <= 3)
    {
        resolutions[index] = 1;
    }
    else
    {
        resolutions[index] = 16;
    }

    stepStates[index] = LOW;

    digitalWrite(nEnPins[index], LOW);
    digitalWrite(dirPins[index], directions[index]);
    if (resolutions[index] == 16)
    {
        digitalWrite(mzeroPins[index], HIGH);
        digitalWrite(monePins[index], HIGH);
    }
    else
    {
        digitalWrite(mzeroPins[index], LOW);
        digitalWrite(monePins[index], LOW);
    }

    prevMotorTimes[index] = 0;
    runnings[index]       = true;
}

void lockAlignment()
{
    alignmentEnabled = false;
}

void unlockAlignment()
{
    prevCoderTime    = 0;
    alignmentEnabled = true;
}

void sendAllStops()
{
    for (int idx = 0; idx < 10; ++idx)
    {
        const uint8_t stopByteCode = stopIndexToByteCode(idx);

        if (stopByteCode == INVALID_STOP_INDEX)
            continue;

        byte buff[4] = {3, 'S', stopByteCode, stopStates[idx]};
        Serial.write(buff, 4);
    }
}

//  ---------------------------------------------------------------------
//                    Reglage Focale et Led
//  ---------------------------------------------------------------------

void sendSpiFocaled(char *buff, int count)
{
    if (count >= 5)
    {
        byte first  = 0;
        byte second = 0;
        bitWrite(first, 7, (buff[2] == 'F' || buff[2] == 'f')); // 0 pour LED et 1 pour Focale
        // bitWrite(first, 6, 0);              // Ca ne sert a rien supprimé par MF et SB le 13/05/2025 apres revu de ce bout de code
        bitWrite(first, 5, 1);              // Gestion du gain    0 = gain de 2           1 = gain de 1
        bitWrite(first, 4, buff[3] == '1'); // ShutDown           0 = output disabled     1 = output enabled

        int value = 0;
        int index = 4;
        while (index < count)
        {
            value = (value * 10) + (buff[index] - '0');
            index++;
        }
        second = value % 256;
        first += value / 256;

        if (buff[1] == 'R' || buff[1] == 'r')
        {
            SPI.transfer(rightSlave, first, SPI_CONTINUE);
            SPI.transfer(rightSlave, second, SPI_LAST);
        }
        else if (buff[1] == 'L' || buff[1] == 'l')
        {
            SPI.transfer(leftSlave, first, SPI_CONTINUE);
            SPI.transfer(leftSlave, second, SPI_LAST);
        }
    }
}

void disableFocaled(char *buff, int count)
{
    if (count >= 2)
    {
        if (buff[1] == 'R' || buff[1] == 'r')
        {
            SPI.transfer(rightSlave, 0x80, SPI_CONTINUE);
            SPI.transfer(rightSlave, 0x00, SPI_LAST);
            SPI.transfer(rightSlave, 0x00, SPI_CONTINUE);
            SPI.transfer(rightSlave, 0x00, SPI_LAST);
        }
        else if (buff[1] == 'L' || buff[1] == 'l')
        {
            SPI.transfer(leftSlave, 0x80, SPI_CONTINUE);
            SPI.transfer(leftSlave, 0x00, SPI_LAST);
            SPI.transfer(leftSlave, 0x00, SPI_CONTINUE);
            SPI.transfer(leftSlave, 0x00, SPI_LAST);
        }
    }
}

//  ---------------------------------------------------------------------
//                    Codeurs lineaires
//  ---------------------------------------------------------------------

void giveCoders()
{
    byte buff[6];

    buff[0] = 5;
    for (int index = 0; index < 7; ++index)
    {
        if (coderChanged[index])
        {
            const uint8_t motorByteCode = motorIndexToAsciiByte(index);

            if (motorByteCode == INVALID_MOTOR_INDEX)
                continue;

            buff[1]             = motorByteCode;
            buff[2]             = coderCounts[index] >> 24;
            buff[3]             = coderCounts[index] >> 16;
            buff[4]             = coderCounts[index] >> 8;
            buff[5]             = coderCounts[index];
            coderChanged[index] = false;
            Serial.write(buff, 6);
        }
    }
}

void lx_coderFront()
{
    boolean a = digitalRead(lx_codA);
    boolean b = digitalRead(lx_codB);

    if (a == b)
        coderCounts[0] += 1;
    else
        coderCounts[0] -= 1;

    coderChanged[0] = true;
}

void ly_coderFront()
{
    boolean a = digitalRead(ly_codA);
    boolean b = digitalRead(ly_codB);

    if (a == b)
        coderCounts[1] += 1;
    else
        coderCounts[1] -= 1;

    coderChanged[1] = true;
}

void rx_coderFront()
{
    boolean a = digitalRead(rx_codA);
    boolean b = digitalRead(rx_codB);

    if (a == b)
        coderCounts[2] += 1;
    else
        coderCounts[2] -= 1;

    coderChanged[2] = true;
}

void ry_coderFront()
{
    boolean a = digitalRead(ry_codA);
    boolean b = digitalRead(ry_codB);

    if (a == b)
        coderCounts[3] += 1;
    else
        coderCounts[3] -= 1;

    coderChanged[3] = true;
}

void px_coderFront()
{
    boolean a = digitalRead(px_codA);
    boolean b = digitalRead(px_codB);

    if (a == b)
        coderCounts[4] += 1;
    else
        coderCounts[4] -= 1;

    coderChanged[4] = true;
}

void py_coderFront()
{
    boolean a = digitalRead(py_codA);
    boolean b = digitalRead(py_codB);

    if (a == b)
        coderCounts[5] += 1;
    else
        coderCounts[5] -= 1;

    coderChanged[5] = true;
}

void pt_coderFront()
{
    boolean a = digitalRead(pt_codA);
    boolean b = digitalRead(pt_codB);

    if (a == b)
        coderCounts[6] += 1;
    else
        coderCounts[6] -= 1;

    coderChanged[6] = true;
}

void resetCodersCount(char *buff, int count)
{
    if (count < 6) // Invalid request
        return;

    const uint8_t idx = asciiByteToMotorIndex(buff[1]);

    if (idx == INVALID_MOTOR_INDEX)
        return;

    coderChanged[idx] = true;
    coderCounts[idx]  = ((long(buff[2]) << 24) + (long(buff[3]) << 16) + (long(buff[4]) << 8) + long(buff[5]));
    giveCoders();
}

//  ---------------------------------------------------------------------
//                    Communication
//  ---------------------------------------------------------------------

void processInstruction(char *buff, uint32_t size);

void serialEvent()
{
    static bool rxReading        = false; // false: Waiting for Length, true: Reading Data
    static uint8_t rxExpectedLen = 0;     // Number of bytes to read
    static int rxIndex           = 0;     // Current buffer position
    static char rxBuffer[64];             // Buffer

    while (Serial.available() > 0)
    {
        byte incoming = Serial.read();

        if (rxReading == 0)
        {
            rxExpectedLen = (int)incoming; // First byte is the length
            rxIndex       = 0;             // Reset to 0

            if (rxExpectedLen > 0 && rxExpectedLen < MAXIMUM_SERIAL_INSTRUCTION_SIZE)
                rxReading = true; // Switch to reading instruction data
            else
                rxReading = false; // If length is 0 or too big, reset (garbage data)
        }
        else if (rxReading == true)
        {
            // Read body bytes
            rxBuffer[rxIndex++] = (char)incoming;

            // Check if packet is complete
            if (rxIndex >= rxExpectedLen) // >= is just to be safe (should never exceed `rxExpectedLen`)
            {
                processInstruction(rxBuffer, rxExpectedLen);
                rxReading = false;
            }
        }
    }
}

void processInstruction(char *buff, uint32_t size)
{
    switch (buff[0])
    {
    case '1':
    {
        if (size > 1)
        {
            const uint8_t idx = asciiByteToMotorIndex(buff[1]);

            if (idx == INVALID_MOTOR_INDEX)
                return;
            if (runnings[idx])
            {
                motorTarget[idx] = -1;
                stopMotor(idx, STOPBY_ORDER_SOFT);
            }
        }
        break;
    }
    case '2':
    {
        if (alignmentEnabled)
            startMotor(buff, size);
        break;
    }
    case '3':
    {
        enableMotor(buff, size);
        break;
    }
    case '4':
    {
        sendSpiFocaled(buff, size);
        break;
    }
    case '5':
    {
        disableFocaled(buff, size);
        break;
    }
    case '7':
    {
        unlockAlignment();
        break;
    }
    case '8':
    {
        lockAlignment();
        break;
    }
    case 'R':
    {
        resetCodersCount(buff, size);
        break;
    }
    case 'T':
    {
        moveToTarget(buff, size);
        break;
    }
    case '?':
    {
        if (size > 1)
        {
            if (buff[1] == 'S')
                sendAllStops();
            else if (buff[1] == 'C')
            {
                for (int idx = 0; idx < 7; ++idx)
                    coderChanged[idx] = true;
                giveCoders();
            }
        }
        else // Request for version
            sendPlainPacketSize((byte *)VERSION, 21);
        break;
    }
    // case 'E': // ECHO Command for testing
    // {
    //     // Responds with the exact same buffer (Loopback)
    //     // buff[0] is 'E', followed by data.
    //     sendPlainPacketSize((byte *)buff, size);
    //     break;
    // }
    default:
        break;
    }
}

void sendPlainPacketSize(byte *message, int count)
{
    byte array[count + 1];

    memcpy(&array[1], message, count);
    array[0] = count;
    Serial.write(array, count + 1);
}
