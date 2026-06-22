#include <SPI.h>

#include "SerialTXHandler.h"
#include "insolation.h"
#include "pins.h"
#include "temperature.h"

namespace Insolation
{
    namespace
    {
        const unsigned int ledVoltageLines[4] = {LED_TENS, LED_OFFSET, LED_TENS_C, LED_OFFSET_C};

        // Active Process States
        unsigned long insolationPrevTime = 0;
        unsigned long insolationTime     = 0;
        unsigned long pauseTime          = 0;
        unsigned long cycleTime          = 0;
        int numberCycles                 = 0;
        int insolationPowerLed           = 0;
        int insolationPowerCouronne      = 0;

        // Feedback multiplexer
        unsigned long sensorPrevTime = 0;
        unsigned int numberLines     = 8;
        byte multiplex               = 0;

        /**
         * @brief Updates the SPI drivers to generate the exact specified output voltages.
         */
        void _sendSPICommand(unsigned long time, unsigned int power, unsigned int couronne)
        {
            if (time > 64800000 || power > 100 || couronne > 100)
                return;

            byte pLeds[2]     = {0, 0};
            byte pCouronne[2] = {0, 0};

            // Protocol specifications: Bit 4 high indicates driver active
            bitWrite(pLeds[0], 4, 1);
            bitWrite(pCouronne[0], 4, 1);

            int valueLed = map(power, 0, 100, 0, 4095);
            pLeds[0] += valueLed / 256;
            pLeds[1] = valueLed % 256;

            int valueCouronne = map(couronne, 0, 100, 0, 4095);
            pCouronne[0] += valueCouronne / 256;
            pCouronne[1] = valueCouronne % 256;

            cycleTime = time;

            SPI.transfer(INSOLEDS, pLeds[0], SPI_CONTINUE);
            SPI.transfer(INSOLEDS, pLeds[1], SPI_LAST);

            SPI.transfer(INSOLCOURONNE, pCouronne[0], SPI_CONTINUE);
            SPI.transfer(INSOLCOURONNE, pCouronne[1], SPI_LAST);
        }

        /**
         * @brief Averages analog voltage across multiple iterations to stabilize reading.
         */
        void _performMoyennage(void)
        {
            int tens[2]   = {0, 0};
            int offset[2] = {0, 0};
            float average = 0.0;

            for (int j = 0; j < 4; ++j)
            {
                average = 0.0;
                for (unsigned i = 0; i < 10; ++i)
                {
                    average += analogRead(ledVoltageLines[j]);
                }
                average /= 10.0;

                if (j % 2 == 0)
                    tens[j / 2] = (int)average;
                else
                    offset[j / 2] = (int)average;
            }

            const byte buff[] = {
                'I', 'V', multiplex,
                lowByte(tens[0] >> 8), lowByte(tens[0]),
                lowByte(offset[0] >> 8), lowByte(offset[0]),
                lowByte(tens[1] >> 8), lowByte(tens[1]),
                lowByte(offset[1] >> 8), lowByte(offset[1])};
            Com::send(serial_packet_t(buff, sizeof(buff)));
        }

        /**
         * @brief Steps the hardware multiplexer bits across the feedback lines.
         */
        void _stepMultiplexerFeedback(void)
        {
            multiplex = (multiplex + 1) % numberLines;
            digitalWrite(GET_A0, bitRead(multiplex, 0));
            digitalWrite(GET_A1, bitRead(multiplex, 1));
            digitalWrite(GET_A2, bitRead(multiplex, 2));
            digitalWrite(GET_A0_C, bitRead(multiplex, 0));
            digitalWrite(GET_A1_C, bitRead(multiplex, 1));
            digitalWrite(GET_A2_C, bitRead(multiplex, 2));
        }

        /**
         * @brief Parses an arbitrary integer dynamically off the buffer, separated by `#`.
         */
        uint32_t _parseValue(char *buff, int &index, int count)
        {
            uint32_t val = 0;
            while (index < count && buff[index] != '#' && buff[index] != 0)
            {
                val = (val * 10) + (buff[index] - '0');
                index++;
            }
            index++; // Step past '#' separator
            return val;
        }
    }

    void setup(void)
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

    void loop(void)
    {
        const uint32_t now = millis();

        // 1. Process Timing Execution
        if (numberCycles > 0 && (unsigned long)(now - insolationPrevTime) >= cycleTime)
        {
            numberCycles--;
            insolationPrevTime = now;

            if (numberCycles == 0)
            {
                stopCycle('E');
            }
            else if (numberCycles % 2 == 0) // Next Active Phase
            {
                _sendSPICommand(insolationTime, insolationPowerLed, insolationPowerCouronne);
            }
            else // Pause Phase
            {
                _sendSPICommand(pauseTime, 0, 0);
            }
        }

        // 2. Feedback Polling
        if (numberCycles > 0 && (unsigned long)(now - sensorPrevTime) >= 100)
        {
            char buff[3] = {'?', 'T', '0'};
            Temperature::checkSensors(buff, 3);
            buff[2] = '1';
            Temperature::checkSensors(buff, 3);

            if (numberCycles % 2 == 0) // Only poll LEDs if active
                _performMoyennage();

            _stepMultiplexerFeedback();
            sensorPrevTime = millis();
        }
    }

    void startCycle(char *buff, int count)
    {
        if (count < 5)
            return;

        int index          = 2;
        unsigned long time = 0, pause = 0;
        unsigned int power = 0, couronne = 0;
        int number = 0;

        multiplex = 0;
        // Reset multiplex lines safely
        digitalWrite(GET_A0, LOW);
        digitalWrite(GET_A1, LOW);
        digitalWrite(GET_A2, LOW);
        digitalWrite(GET_A0_C, LOW);
        digitalWrite(GET_A1_C, LOW);
        digitalWrite(GET_A2_C, LOW);

        if (buff[1] == 'C' || buff[1] == 'c')
        {
            time     = _parseValue(buff, index, count);
            power    = _parseValue(buff, index, count);
            couronne = _parseValue(buff, index, count);
            number   = 1;
        }
        else if (buff[1] == 'F' || buff[1] == 'f')
        {
            number   = _parseValue(buff, index, count);
            time     = _parseValue(buff, index, count);
            pause    = _parseValue(buff, index, count);
            power    = _parseValue(buff, index, count);
            couronne = _parseValue(buff, index, count);
        }

        if (number > 0)
        {
            multiplex = 0;

            digitalWrite(disableLDAC, LOW);
            digitalWrite(RELAYPIN, HIGH);

            digitalWrite(GET_EN, HIGH);
            digitalWrite(GET_EN_C, HIGH);

            delay(100); // Allow hardware lines to settle

            insolationTime          = time;
            pauseTime               = pause;
            insolationPowerLed      = power;
            insolationPowerCouronne = couronne;
            numberCycles            = number * 2;

            insolationPrevTime = millis();
            sensorPrevTime     = millis();

            _sendSPICommand(insolationTime, insolationPowerLed, insolationPowerCouronne);
        }
    }

    void stopCycle(char code)
    {
        numberCycles            = 0;
        insolationTime          = 0;
        pauseTime               = 0;
        cycleTime               = 0;
        insolationPowerLed      = 0;
        insolationPowerCouronne = 0;

        SPI.transfer(INSOLEDS, 0x00, SPI_CONTINUE);
        SPI.transfer(INSOLEDS, 0x00, SPI_LAST);

        SPI.transfer(INSOLCOURONNE, 0x00, SPI_CONTINUE);
        SPI.transfer(INSOLCOURONNE, 0x00, SPI_LAST);

        digitalWrite(RELAYPIN, LOW);
        digitalWrite(disableLDAC, HIGH);
        digitalWrite(GET_EN, LOW);
        digitalWrite(GET_EN_C, LOW);

        const byte buff[] = {'I', code};
        Com::send(serial_packet_t(buff, sizeof(buff)));
    }

    void interruptExposure(void)
    {
        if (numberCycles > 0)
            stopCycle('I');
    }

    unsigned long getCycleTime()
    {
        return cycleTime;
    }
    int getNumberCycles()
    {
        return numberCycles;
    }
}
