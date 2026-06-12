#include "SerialTXHandler.h"

namespace SerialTXHandler
{
    static AtomicQueue<serial_packet_t, 64> msgQueue;
    static serial_packet_t dmaBuffers[2]; // Create 2 separate buffers in RAM so the two DMA lanes don't share memory
    static uint8_t activeBufferIdx = 0;

    bool push(const serial_packet_t &packet)
    {
        return msgQueue.push(packet);
    }

    void processPackets(void)
    {
        while (!msgQueue.isEmpty())
        {
            if (UART->UART_TCR == 0) // Is the "Current" hardware lane idle
            {
                if (msgQueue.pop(dmaBuffers[activeBufferIdx]))
                {
                    UART->UART_TPR  = (uint32_t)dmaBuffers[activeBufferIdx].msg;
                    UART->UART_TCR  = dmaBuffers[activeBufferIdx].size;
                    UART->UART_PTCR = UART_PTCR_TXTEN;

                    activeBufferIdx ^= 1; // Toggle the index
                }
            }
            else if (UART->UART_TNCR == 0) // Is the "Next" hardware lane idle
            {
                if (msgQueue.pop(dmaBuffers[activeBufferIdx]))
                {
                    UART->UART_TNPR = (uint32_t)dmaBuffers[activeBufferIdx].msg;
                    UART->UART_TNCR = dmaBuffers[activeBufferIdx].size;

                    activeBufferIdx ^= 1; // Toggle the index
                }
            }
            else // Both lanes (current and next) are used
                break;
        }
    }
}
