#include "focalLed.h"
#include "pins.h"
#include <Arduino.h>
#include <SPI.h>

namespace FocalLed
{
    void setup()
    {
        // Configure hardware SPI for the LED/Focal driver boards.
        // SPI_MODE3 (CPOL=1, CPHA=1) and Clock Divider 21 (~4MHz on 84MHz bus).
        SPI.begin(LEFT_SLAVE);
        SPI.begin(RIGHT_SLAVE);
        SPI.setDataMode(LEFT_SLAVE, SPI_MODE3);
        SPI.setBitOrder(LEFT_SLAVE, MSBFIRST);
        SPI.setDataMode(RIGHT_SLAVE, SPI_MODE3);
        SPI.setBitOrder(RIGHT_SLAVE, MSBFIRST);
        SPI.setClockDivider(LEFT_SLAVE, 21);
        SPI.setClockDivider(RIGHT_SLAVE, 21);
    }

    void sendSPI(char *buff, int count)
    {
        if (count >= 5)
        {
            byte first  = 0;
            byte second = 0;

            // Build the SPI control byte bit-by-bit based on protocol specs:
            // Bit 7: Target selection (0 = LED, 1 = Focale)
            bitWrite(first, 7, (buff[2] == 'F' || buff[2] == 'f'));
            // Bit 5: Gain management (0 = gain of 2, 1 = gain of 1)
            bitWrite(first, 5, 1);
            // Bit 4: Shutdown (0 = output disabled, 1 = output enabled)
            bitWrite(first, 4, buff[3] == '1');

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
                SPI.transfer(RIGHT_SLAVE, first, SPI_CONTINUE);
                SPI.transfer(RIGHT_SLAVE, second, SPI_LAST);
            }
            else if (buff[1] == 'L' || buff[1] == 'l')
            {
                SPI.transfer(LEFT_SLAVE, first, SPI_CONTINUE);
                SPI.transfer(LEFT_SLAVE, second, SPI_LAST);
            }
        }
    }

    void disable(char *buff, int count)
    {
        if (count >= 2)
        {
            if (buff[1] == 'R' || buff[1] == 'r')
            {
                SPI.transfer(RIGHT_SLAVE, 0x80, SPI_CONTINUE);
                SPI.transfer(RIGHT_SLAVE, 0x00, SPI_LAST);
                SPI.transfer(RIGHT_SLAVE, 0x00, SPI_CONTINUE);
                SPI.transfer(RIGHT_SLAVE, 0x00, SPI_LAST);
            }
            else if (buff[1] == 'L' || buff[1] == 'l')
            {
                SPI.transfer(LEFT_SLAVE, 0x80, SPI_CONTINUE);
                SPI.transfer(LEFT_SLAVE, 0x00, SPI_LAST);
                SPI.transfer(LEFT_SLAVE, 0x00, SPI_CONTINUE);
                SPI.transfer(LEFT_SLAVE, 0x00, SPI_LAST);
            }
        }
    }
}
