#include "encoder.h"
#include "SerialTXHandler.h"
#include "pins.h"

// Local scope
namespace
{
    // State variables
    volatile int32_t coderCounts[ENCODERS_COUNT] = {0, 0, 0, 0, 0};
    volatile bool coderChanged[ENCODERS_COUNT]   = {false, false, false, false, false};

    void sendSingleCoder(uint8_t encoderIdx)
    {
        // Save the current interrupt state (PRIMASK) and globally disable interrupts.
        // We do this to create a Critical Section. While a 32-bit read is natively atomic
        // on the Cortex-M3, reading the value AND clearing the `coderChanged` flag must
        // occur uninterrupted to prevent race conditions with the encoder ISRs.
        const uint32_t primask = __get_PRIMASK();

        __disable_irq();
        coderChanged[encoderIdx] = false;
        int32_t currentCount     = coderCounts[encoderIdx]; // Atomic value copy
        __set_PRIMASK(primask);

        // Convert 0-indexed ID back to ASCII '1'-'5' for serial protocol
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
        // Standard Quadrature decoding logic. Called strictly from hardware ISR context.
        if (encoderAValue == encoderBValue)
            coderCounts[encoderIdx] += 1;
        else
            coderCounts[encoderIdx] -= 1;
        coderChanged[encoderIdx] = true;
    }

    // ISR callbacks
    void encoderISRHandlerMALG()
    {
        updateEncoder(Z_LEFT, Z_LEFT_ENC_A_VAL, Z_LEFT_ENC_B_VAL);
    }
    void encoderISRHandlerMALD()
    {
        updateEncoder(Z_RIGHT, Z_RIGHT_ENC_A_VAL, Z_RIGHT_ENC_B_VAL);
    }
    void encoderISRHandlerMALA()
    {
        updateEncoder(Z_BACK, Z_BACK_ENC_A_VAL, Z_BACK_ENC_B_VAL);
    }
    void encoderISRHandlerTPM()
    {
        updateEncoder(MASK_DRAWER, MASK_ENC_A_VAL, MASK_ENC_B_VAL);
    }
    void encoderISRHandlerTPW()
    {
        updateEncoder(WAFER_DRAWER, WAFER_ENC_A_VAL, WAFER_ENC_B_VAL);
    }
}

// Public scope
namespace Encoders
{
    void setup()
    {
        const uint8_t encoderDuePins[ENCODERS_COUNT][2] = {
            {Z_LEFT_ENC_A_PIN, Z_LEFT_ENC_B_PIN},
            {Z_RIGHT_ENC_A_PIN, Z_RIGHT_ENC_B_PIN},
            {Z_BACK_ENC_A_PIN, Z_BACK_ENC_B_PIN},
            {MASK_ENC_A_PIN, MASK_ENC_B_PIN},
            {WAFER_ENC_A_PIN, WAFER_ENC_B_PIN},
        };

        for (uint8_t index = 0; index < ENCODERS_COUNT; ++index)
        {
            pinMode(encoderDuePins[index][0], INPUT);
            pinMode(encoderDuePins[index][1], INPUT);
        }

        // Hardware Interrupts
        attachInterrupt(digitalPinToInterrupt(Z_LEFT_ENC_A_PIN), &encoderISRHandlerMALG, CHANGE);
        attachInterrupt(digitalPinToInterrupt(Z_RIGHT_ENC_A_PIN), &encoderISRHandlerMALD, CHANGE);
        attachInterrupt(digitalPinToInterrupt(Z_BACK_ENC_A_PIN), &encoderISRHandlerMALA, CHANGE);
        attachInterrupt(digitalPinToInterrupt(MASK_ENC_A_PIN), &encoderISRHandlerTPM, CHANGE);
        attachInterrupt(digitalPinToInterrupt(WAFER_ENC_A_PIN), &encoderISRHandlerTPW, CHANGE);
    }

    void loop()
    {
        static uint32_t prevTime = 0;
        uint32_t now             = millis();

        if (now - prevTime >= 50) // Send encoders value every 50ms
        {
            sendChanged();
            prevTime = now;
        }
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

        // Extract ID directly from buffer, converting ASCII '1'-'5' to 0-4
        const uint8_t idx = buff[1] - '1';
        if (idx >= ENCODERS_COUNT)
            return;

        // Reconstruct the 32-bit signed integer from the Big-Endian serial payload
        coderCounts[idx] =
            ((int32_t)((uint8_t)buff[2]) << 24) |
            ((int32_t)((uint8_t)buff[3]) << 16) |
            ((int32_t)((uint8_t)buff[4]) << 8) |
            (uint8_t)buff[5];

        coderChanged[idx] = true;
        sendSingleCoder(idx);
    }

    volatile int32_t *getCountPtr(MotorId id)
    {
        if (id >= ENCODERS_COUNT)
            return nullptr; // Out of bounds
        return &coderCounts[id];
    }

    int32_t getValue(MotorId id)
    {
        if (id >= ENCODERS_COUNT)
            return INT32_MIN;
        return coderCounts[id];
    }
}
