#include <SPI.h>

#include "SerialTXHandler.h"
#include "insolation.h"
#include "temperature.h"

//  Variables d'insolation
static unsigned int ledVoltageLine[4] = {LED_TENS, LED_OFFSET, LED_TENS_C, LED_OFFSET_C};
static unsigned long insoPrevTime     = 0;
static unsigned long insolationTime   = 0;
unsigned long pauseTime               = 0;
unsigned long cycleTime               = 0;
int numberCycles                      = 0;
int insolationPowerLed                = 0;
int insolationPowerCouronne           = 0;
unsigned long sensorPrevTime          = 0;
unsigned int numberLines              = 8;
byte multiplex                        = 0;

void setupInsolation()
{
    SPI.begin(INSOLEDS);
    SPI.setDataMode(INSOLEDS, SPI_MODE3);
    SPI.setBitOrder(INSOLEDS, MSBFIRST);
    SPI.setClockDivider(INSOLEDS, 21);
    SPI.begin(INSOLCOURONNE);
    SPI.setDataMode(INSOLCOURONNE, SPI_MODE3);
    SPI.setBitOrder(INSOLCOURONNE, MSBFIRST);
    SPI.setClockDivider(INSOLCOURONNE, 21);

    pinMode(RELAYPIN, OUTPUT);
    digitalWrite(RELAYPIN, LOW);

    pinMode(disableLDAC, OUTPUT);
    digitalWrite(disableLDAC, HIGH);

    pinMode(GET_EN, OUTPUT);
    pinMode(GET_A0, OUTPUT);
    pinMode(GET_A1, OUTPUT);
    pinMode(GET_A2, OUTPUT);
    digitalWrite(GET_EN, LOW);
    digitalWrite(GET_A0, LOW);
    digitalWrite(GET_A1, LOW);
    digitalWrite(GET_A2, LOW);

    pinMode(GET_EN_C, OUTPUT);
    pinMode(GET_A0_C, OUTPUT);
    pinMode(GET_A1_C, OUTPUT);
    pinMode(GET_A2_C, OUTPUT);
    digitalWrite(GET_EN_C, LOW);
    digitalWrite(GET_A0_C, LOW);
    digitalWrite(GET_A1_C, LOW);
    digitalWrite(GET_A2_C, LOW);
}

void loopInsolation()
{
    //  Execution de l'insolation
    if (numberCycles > 0 && (unsigned long)(millis() - insoPrevTime) >= cycleTime)
    {
        numberCycles -= 1;
        insoPrevTime = millis();
        if (numberCycles == 0)
        {
            stopInsolation('E');
        }
        else if (numberCycles % 2 == 0)
        {
            startInsolationCycle(insolationTime, insolationPowerLed, insolationPowerCouronne);
        }
        else
        {
            startInsolationCycle(pauseTime, 0, 0);
        }
    }
    //  Verification des capteurs
    if (numberCycles > 0 && (unsigned long)(millis() - sensorPrevTime) >= 100)
    {
        /// checkFans();
        // checkTemperatures
        char buff[3] = {'?', 'T', '0'};
        checkTemperature(buff, 3);
        buff[2] = '1';
        checkTemperature(buff, 3);

        //  Tension des LEDS
        if (numberCycles % 2 == 0)
        {
            moyennageInsol();
        }
        multiplexFeedback();
        sensorPrevTime = millis();
    }
}

void multiplexFeedback()
{
    multiplex = (multiplex + 1) % numberLines;
    digitalWrite(GET_A0, bitRead(multiplex, 0));
    digitalWrite(GET_A1, bitRead(multiplex, 1));
    digitalWrite(GET_A2, bitRead(multiplex, 2));
    digitalWrite(GET_A0_C, bitRead(multiplex, 0));
    digitalWrite(GET_A1_C, bitRead(multiplex, 1));
    digitalWrite(GET_A2_C, bitRead(multiplex, 2));
}

void initInsolation(char *buff, int count)
{
    if (count >= 5)
    {
        int index(2);
        unsigned long time(0);
        unsigned long pause(0);
        unsigned int power(0);
        unsigned int couronne(0);
        int number(0);

        multiplex = 0;
        digitalWrite(GET_A0, 0);
        digitalWrite(GET_A1, 0);
        digitalWrite(GET_A2, 0);

        digitalWrite(GET_A0_C, 0);
        digitalWrite(GET_A1_C, 0);
        digitalWrite(GET_A2_C, 0);

        if (buff[1] == 'C' || buff[1] == 'c')
        {
            while (index < count && buff[index] != '#' && buff[index] != 0)
            {
                time = (time * 10) + (unsigned int)(buff[index] - '0');
                index++;
            }
            index += 1;

            while (index < count && buff[index] != '#' && buff[index] != 0)
            {
                power = (power * 10) + (unsigned int)(buff[index] - '0');
                index++;
            }
            index += 1;

            while (index < count && buff[index] != 0)
            {
                couronne = (couronne * 10) + (unsigned int)(buff[index] - '0');
                index++;
            }
            number = 1;
        }
        else if (buff[1] == 'F' || buff[1] == 'f')
        {
            while (index < count && buff[index] != '#' && buff[index] != 0)
            {
                number = (number * 10) + int(buff[index] - '0');
                index++;
            }
            index += 1;

            while (index < count && buff[index] != '#' && buff[index] != 0)
            {
                time = (time * 10) + (unsigned int)(buff[index] - '0');
                index++;
            }
            index += 1;

            while (index < count && buff[index] != '#' && buff[index] != 0)
            {
                pause = (pause * 10) + (unsigned int)(buff[index] - '0');
                index++;
            }
            index += 1;

            while (index < count && buff[index] != '#' && buff[index] != 0)
            {
                power = (power * 10) + (unsigned int)(buff[index] - '0');
                index++;
            }
            index += 1;

            while (index < count && buff[index] != 0)
            {
                couronne = (couronne * 10) + (unsigned int)(buff[index] - '0');
                index++;
            }
        }
        if (number > 0)
        {
            multiplex = 0;

            digitalWrite(disableLDAC, LOW);
            digitalWrite(RELAYPIN, HIGH);
            digitalWrite(GET_EN, HIGH);
            digitalWrite(GET_A0, LOW);
            digitalWrite(GET_A1, LOW);
            digitalWrite(GET_A2, LOW);

            digitalWrite(GET_EN_C, HIGH);
            digitalWrite(GET_A0_C, LOW);
            digitalWrite(GET_A1_C, LOW);
            digitalWrite(GET_A2_C, LOW);

            delay(100);
            insolationTime          = time;
            pauseTime               = pause;
            insolationPowerLed      = power;
            insolationPowerCouronne = couronne;
            numberCycles            = number * 2;
            insoPrevTime            = millis();
            sensorPrevTime          = millis();
            startInsolationCycle(insolationTime, insolationPowerLed, insolationPowerCouronne);
        }
    }
}

void startInsolationCycle(unsigned long time, unsigned int power, unsigned int couronne)
{
    // Note : La valeur 64800000 ms correspond à 18h mais est limité dans le soft à 1h 59min
    if (time <= 64800000 && (power >= 0 && power <= 100) && (couronne >= 0 && couronne <= 100))
    {
        // byte first = 0;
        // byte second = 0;
        // bitWrite(first, 7, 0);
        // bitWrite(first, 6, 0);
        // bitWrite(first, 5, 0);
        // bitWrite(first, 4, 1);

        // // (power - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        // int value = map(power, 0, 100, 0, 4095);
        // first += value / 256;
        // second = value % 256;

        // // Couronne
        // int valueCouronne = map(couronne, 0, 100, 0, 4095);
        // first += valueCouronne / 256;
        // second = valueCouronne % 256;

        byte pLeds[2]     = {0, 0};
        byte pCouronne[2] = {0, 0};

        bitWrite(pLeds[0], 7, 0);
        bitWrite(pLeds[0], 6, 0);
        bitWrite(pLeds[0], 5, 0);
        bitWrite(pLeds[0], 4, 1);

        bitWrite(pCouronne[0], 7, 0);
        bitWrite(pCouronne[0], 6, 0);
        bitWrite(pCouronne[0], 5, 0);
        bitWrite(pCouronne[0], 4, 1);

        // (power - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        int value = map(power, 0, 100, 0, 4095);
        pLeds[0] += value / 256;
        pLeds[1] = value % 256;

        // Couronne
        int valueCouronne = map(couronne, 0, 100, 0, 4095);
        pCouronne[0] += valueCouronne / 256;
        pCouronne[1] = valueCouronne % 256;

        cycleTime = time;
        // 1ere: Pin, 2eme: Octét à envoyer, 3eme: Flag
        SPI.transfer(INSOLEDS, pLeds[0], SPI_CONTINUE);
        SPI.transfer(INSOLEDS, pLeds[1], SPI_LAST);

        SPI.transfer(INSOLCOURONNE, pCouronne[0], SPI_CONTINUE);
        SPI.transfer(INSOLCOURONNE, pCouronne[1], SPI_LAST);
    }
}

void stopInsolation(char code)
{
    numberCycles            = 0;
    insolationTime          = 0;
    pauseTime               = 0;
    cycleTime               = 0;
    insolationPowerLed      = 0;
    insolationPowerCouronne = 0;
    insoPrevTime            = 0;
    sensorPrevTime          = 0;

    SPI.transfer(INSOLEDS, 0x00, SPI_CONTINUE);
    SPI.transfer(INSOLEDS, 0x00, SPI_LAST);

    SPI.transfer(INSOLCOURONNE, 0x00, SPI_CONTINUE);
    SPI.transfer(INSOLCOURONNE, 0x00, SPI_LAST);

    digitalWrite(RELAYPIN, LOW);
    digitalWrite(disableLDAC, HIGH);
    digitalWrite(GET_EN, LOW);
    digitalWrite(GET_EN_C, LOW);

    byte buff[3];
    buff[0] = 2;
    buff[1] = 'I';
    buff[2] = code;

    Com::send(serial_packet_t((uint8_t *)&buff[1], 2));
}

void interruptExposure()
{
    if (numberCycles > 0)
    {
        stopInsolation('I');
    }
}

void receiveNumberLines(byte *buff, int count)
{
    if (count >= 2)
    {
        int index(1);
        unsigned int number(0);

        while (index < count && buff[index] >= '0' && buff[index] <= '9')
        {
            number = (10 * number) + (unsigned int)(buff[index] - '0');
            index++;
        }

        if (number >= 1 && number <= 8)
            numberLines = number;
    }
}

void moyennageInsol(void)
{
    int tens[2];
    int offset[2];
    float moyenne = 0.0;

    for (int j = 0; j < 4; ++j)
    {
        for (unsigned i = 0; i < 10; ++i)
        {
            moyenne += analogRead(ledVoltageLine[j]);
        }
        moyenne /= 10.0;
        if (j % 2 == 0)
        {
            tens[j] = (int)moyenne;
        }
        else
        {
            offset[j] = (int)moyenne;
        }
    }

    byte buff[12] = {11, 'I', 'V', multiplex, lowByte(tens[0] >> 8), lowByte(tens[0]), lowByte(offset[0] >> 8), lowByte(offset[0]),
                     lowByte(tens[1] >> 8), lowByte(tens[1]), lowByte(offset[1] >> 8), lowByte(offset[1])};
    Com::send(serial_packet_t((uint8_t *)&buff[1], 11));
}

unsigned long getCycleTime()
{
    return cycleTime;
}

int getNumberCycles()
{
    return numberCycles;
}
