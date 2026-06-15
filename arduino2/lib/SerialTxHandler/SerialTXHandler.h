#ifndef DMA_UART_H
#define DMA_UART_H

#include "AtomicQueue.hpp"

typedef struct serial_packet_s {
    uint8_t msg[64]; // Max 64 chars per message
    uint16_t size;   // The actual number of bytes used in the message to send

    serial_packet_s(const uint8_t *data, uint16_t dataCount)
    {
        size   = dataCount + 1;
        msg[0] = dataCount;
        if (dataCount <= 63)
            memcpy(&msg[1], data, dataCount);
    }

    serial_packet_s() : size(0)
    {
    }
} serial_packet_t;

namespace SerialTXHandler
{
    bool push(const serial_packet_t &packet);
    // alias for the `push` function
    inline bool send(const serial_packet_t &packet)
    {
        return push(packet);
    }

    void processPackets(void);
}

namespace Com = SerialTXHandler;

#endif // DMA_UART_H
