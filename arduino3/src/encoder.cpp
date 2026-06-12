#include <Arduino.h>

#include "encoder.h"
#include "pins.h"
#include "serialTxHandler.h"

#define MALG_index 0 // ASCII '1' - '1' = 0
#define MALD_index 1 // ASCII '2' - '1' = 1
#define MALA_index 2 // ASCII '3' - '1' = 2
#define TPM_index  3 // ASCII '4' - '1' = 3
#define TPW_index  4 // ASCII '5' - '1' = 4

// Local scope
namespace
{
    // State variables
    volatile int32_t coderCounts[ENCODERS_COUNT] = {0, 0, 0, 0, 0};
    volatile bool coderChanged[ENCODERS_COUNT]   = {false, false, false, false, false};

    void sendSingleCoder(uint8_t encoderIdx)
    {
        const uint32_t primask = __get_PRIMASK();

        __disable_irq();
        coderChanged[encoderIdx] = false;
        int32_t currentCount     = coderCounts[encoderIdx]; // value copy
        __set_PRIMASK(primask);

        const uint8_t buffer[] = {
            (uint8_t)(encoderIdx + '1'),
            (uint8_t)((currentCount >> 24) & 0xFF),
            (uint8_t)((currentCount >> 16) & 0xFF),
            (uint8_t)((currentCount >> 8) & 0xFF),
            (uint8_t)(currentCount & 0xFF),
        };

        Com::send(serial_packet_t(buffer, sizeof(buffer)));
    }

    void updateEncoder(uint8_t encoderIdx, uint8_t encoderAValue, uint8_t encoderBValue)
    {
        if (encoderAValue == encoderBValue)
            coderCounts[encoderIdx] += 1;
        else
            coderCounts[encoderIdx] -= 1;
        coderChanged[encoderIdx] = true;
    }

    // ISR callbacks

    void encoderISRHandlerMALG(void)
    {
        updateEncoder(MALG_index, MALG_codA_VAL, MALG_codB_VAL);
    }

    void encoderISRHandlerMALD(void)
    {
        updateEncoder(MALD_index, MALD_codA_VAL, MALD_codB_VAL);
    }

    void encoderISRHandlerMALA(void)
    {
        updateEncoder(MALA_index, MALA_codA_VAL, MALA_codB_VAL);
    }

    void encoderISRHandlerTPM(void)
    {
        updateEncoder(TPM_index, MCM_codA_VAL, MCM_codB_VAL);
    }

    void encoderISRHandlerTPW(void)
    {
        updateEncoder(TPW_index, MCW_codA_VAL, MCW_codB_VAL);
    }
}

// Public scope
namespace Encoders
{
    void setup()
    {
        const uint8_t encoderDuePins[ENCODERS_COUNT][2] = {
            {MALG_codA, MALG_codB},
            {MALD_codA, MALD_codB},
            {MALA_codA, MALA_codB},
            {MCM_codA, MCM_codB},
            {MCW_codA, MCW_codB},
        };

        for (uint8_t index = 0; index < ENCODERS_COUNT; ++index)
        {
            pinMode(encoderDuePins[index][0], INPUT);
            pinMode(encoderDuePins[index][1], INPUT);
        }

        // Interruptions - use digitalPinToInterrupt to map pin -> interrupt num
        attachInterrupt(digitalPinToInterrupt(MALG_codA), &encoderISRHandlerMALG, CHANGE);
        attachInterrupt(digitalPinToInterrupt(MALD_codA), &encoderISRHandlerMALD, CHANGE);
        attachInterrupt(digitalPinToInterrupt(MALA_codA), &encoderISRHandlerMALA, CHANGE);
        attachInterrupt(digitalPinToInterrupt(MCM_codA), &encoderISRHandlerTPM, CHANGE);
        attachInterrupt(digitalPinToInterrupt(MCW_codA), &encoderISRHandlerTPW, CHANGE);
    }

    void sendAll()
    {
        for (uint8_t idx = 0; idx < ENCODERS_COUNT; ++idx)
            sendSingleCoder(idx);
    }

    void sendChanged(void)
    {
        for (uint8_t idx = 0; idx < ENCODERS_COUNT; ++idx)
        {
            if (coderChanged[idx])
                sendSingleCoder(idx);
        }
    }

    void resetCount(char *buff, int count)
    {
        if (count != 6)
            return;

        const uint8_t idx = buff[1] - '1';

        coderCounts[idx] =
            ((int32_t)((uint8_t)buff[2]) << 24) |
            ((int32_t)((uint8_t)buff[3]) << 16) |
            ((int32_t)((uint8_t)buff[4]) << 8) |
            (uint8_t)buff[5];
        coderChanged[idx] = true;
        sendSingleCoder(idx); // Send update through serial
    }

    volatile int32_t *getCountPtr(MotorId id)
    {
        const uint8_t idx = id - '1';

        if (idx >= ENCODERS_COUNT)
            return nullptr; // Out of bounds
        return &coderCounts[idx];
    }

    int32_t getValue(MotorId id)
    {
        const uint8_t idx = id - '1';

        if (idx > 4)
            return INT32_MIN; // Error value
        return coderCounts[idx];
    }
}
