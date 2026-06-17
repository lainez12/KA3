#ifndef FOCALLED_H
#define FOCALLED_H

namespace FocalLed
{
    /**
     * @brief Initializes the SPI bus for communication with the Left and Right
     * Focal/LED control boards.
     */
    void setup();

    /**
     * @brief Parses a serial command and transmits the corresponding SPI payload
     * to adjust the brightness/focus of the target board.
     *
     * @param buff Pointer to the raw serial payload.
     * @param count Total byte length of the payload.
     */
    void sendSPI(char *buff, int count);

    /**
     * @brief Sends a shutdown command over SPI to disable the Focal/LED outputs.
     *
     * @param buff Pointer to the raw serial payload.
     * @param count Total byte length of the payload.
     */
    void disable(char *buff, int count);
}

#endif // FOCALLED_H
