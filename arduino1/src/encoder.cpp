#include "encoder.h"
#include "SerialTXHandler.h"
#include "pins.h"
#include "utils.h"

namespace
{
    volatile int32_t coderCounts[ENCODERS_COUNT] = {0, 0, 0, 0, 0, 0, 0};
    volatile bool coderChanged[ENCODERS_COUNT]   = {false, false, false, false, false, false, false};

    void sendSingleCoder(uint8_t encoderIdx)
    {
        // Save the current interrupt state (PRIMASK) and globally disable interrupts.
        // Reading the value AND clearing the `coderChanged` flag must occur uninterrupted
        // to prevent race conditions with the encoder ISRs.
        const uint32_t primask = __get_PRIMASK();

        __disable_irq();
        coderChanged[encoderIdx] = false;
        int32_t currentCount     = coderCounts[encoderIdx];
        __set_PRIMASK(primask);

        const uint8_t motorByteCode = KUtils::motorIndexToByteCode(encoderIdx);
        if (motorByteCode == INVALID_MOTOR_INDEX)
            return;

        const uint8_t buffer[] = {
            motorByteCode,
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

    void encoderISR_lx()
    {
        updateEncoder(0, LEFT_CAM_X_ENC_A_VAL, LEFT_CAM_X_ENC_B_VAL);
    }
    void encoderISR_ly()
    {
        updateEncoder(1, LEFT_CAM_Y_ENC_A_VAL, LEFT_CAM_Y_ENC_B_VAL);
    }
    void encoderISR_rx()
    {
        updateEncoder(2, RIGHT_CAM_X_ENC_A_VAL, RIGHT_CAM_X_ENC_B_VAL);
    }
    void encoderISR_ry()
    {
        updateEncoder(3, RIGHT_CAM_Y_ENC_A_VAL, RIGHT_CAM_Y_ENC_B_VAL);
    }
    void encoderISR_px()
    {
        updateEncoder(4, STAGE_X_ENC_A_VAL, STAGE_X_ENC_B_VAL);
    }
    void encoderISR_py()
    {
        updateEncoder(5, STAGE_Y_ENC_A_VAL, STAGE_Y_ENC_B_VAL);
    }
    void encoderISR_pt()
    {
        updateEncoder(6, STAGE_TH_ENC_A_VAL, STAGE_TH_ENC_B_VAL);
    }
}

namespace Encoders
{
    void setup()
    {
        const uint8_t pins[ENCODERS_COUNT][2] = {
            {LEFT_CAM_X_ENC_A_PIN, LEFT_CAM_X_ENC_B_PIN},
            {LEFT_CAM_Y_ENC_A_PIN, LEFT_CAM_Y_ENC_B_PIN},
            {RIGHT_CAM_X_ENC_PIN_A, RIGHT_CAM_X_ENC_PIN_B},
            {RIGHT_CAM_Y_ENC_A_PIN, RIGHT_CAM_Y_ENC_B_PIN},
            {STAGE_X_ENC_A_PIN, STAGE_X_ENC_B_PIN},
            {STAGE_Y_ENC_A_PIN, STAGE_Y_ENC_B_PIN},
            {STAGE_TH_ENC_A_PIN, STAGE_TH_ENC_B_PIN}};

        for (uint8_t i = 0; i < ENCODERS_COUNT; ++i)
        {
            pinMode(pins[i][0], INPUT);
            pinMode(pins[i][1], INPUT);
        }

        attachInterrupt(digitalPinToInterrupt(LEFT_CAM_X_ENC_A_PIN), &encoderISR_lx, CHANGE);
        attachInterrupt(digitalPinToInterrupt(LEFT_CAM_Y_ENC_A_PIN), &encoderISR_ly, CHANGE);
        attachInterrupt(digitalPinToInterrupt(RIGHT_CAM_X_ENC_PIN_A), &encoderISR_rx, CHANGE);
        attachInterrupt(digitalPinToInterrupt(RIGHT_CAM_Y_ENC_A_PIN), &encoderISR_ry, CHANGE);
        attachInterrupt(digitalPinToInterrupt(STAGE_X_ENC_A_PIN), &encoderISR_px, CHANGE);
        attachInterrupt(digitalPinToInterrupt(STAGE_Y_ENC_A_PIN), &encoderISR_py, CHANGE);
        attachInterrupt(digitalPinToInterrupt(STAGE_TH_ENC_A_PIN), &encoderISR_pt, CHANGE);
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

    void sendChanged()
    {
        for (uint8_t idx = 0; idx < ENCODERS_COUNT; ++idx)
        {
            if (coderChanged[idx])
                sendSingleCoder(idx);
        }
    }

    void resetCount(char *buff, int count)
    {
        if (count < 6)
            return;

        const uint8_t idx = KUtils::motorByteCodeToIndex(buff[1]);

        if (idx == INVALID_MOTOR_INDEX || idx >= ENCODERS_COUNT)
            return;

        coderCounts[idx]  = ((int32_t)((uint8_t)buff[2]) << 24) |
                            ((int32_t)((uint8_t)buff[3]) << 16) |
                            ((int32_t)((uint8_t)buff[4]) << 8) |
                            (uint8_t)buff[5];
        coderChanged[idx] = true;
        sendSingleCoder(idx);
    }

    volatile int32_t *getCountPtr(uint8_t idx)
    {
        if (idx >= ENCODERS_COUNT)
            return nullptr;
        return &coderCounts[idx];
    }

    int32_t getValue(uint8_t idx)
    {
        if (idx >= ENCODERS_COUNT)
            return INT32_MIN;
        return coderCounts[idx];
    }
}
