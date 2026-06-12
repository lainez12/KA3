#include <Arduino.h>

#include "drawer.h"
#include "encoder.h"
#include "pins.h"
#include "serialTxHandler.h"
#include "timerPriorities.h"

#define DRAWERS_STOPS_COUNT 7
#define FREQ_TARGET_MOTOR   1000
// Communications related
#define INDEX_ASCII_MOTOR_TPM '4'
#define COM_STOP_IDX_OFFSET   6
#define COM_MOTOR_IDX_OFFSET  4
// Positive/negative stops indices
#define INDEX_CM0 0
#define INDEX_CM3 3
#define INDEX_CW0 4
#define INDEX_CW2 6

const uint8_t stopPinsDrawer[DRAWERS_STOPS_COUNT] = {
    BMCM0, // CM0
    BMCM1, // CM1
    BMCM2, // CM2
    BMCM3, // CM3
    BMCW0, // CW0
    BMCW1, // CW1
    BMCW2  // CW2
};
volatile bool stopStatesDrawer[DRAWERS_STOPS_COUNT] = {false, false, false, false, false, false, false};

namespace
{
    void _setConveyorStepperResolution(StepFraction frac, uint32_t *resolutionPins);

    DueStepper maskMotor(
        1, // Timer id
        due_stepper_pins_t{
            .enable           = MCM_nEn,
            .step             = MCM_STEP,
            .direction        = MCM_DIR,
            .resolution       = (uint32_t[2]){MCM_M0, MCM_M1},
            .resolutionSetter = &_setConveyorStepperResolution,
        });

    DueStepper waferMotor(
        2, // Timer id
        due_stepper_pins_t{
            .enable           = MCW_nEn,
            .step             = MCW_STEP,
            .direction        = MCW_DIR,
            .resolution       = (uint32_t[2]){MCW_M0, MCW_M1},
            .resolutionSetter = &_setConveyorStepperResolution,
        });

    DueStepper &_getMotor(uint8_t idx)
    {
        if (idx == 0)
            return maskMotor;
        return waferMotor;
    }

    void _startMotor(uint8_t motorIdx, uint32_t freq, uint8_t direction, uint8_t resolution, uint32_t numberSteps)
    {
        if (motorIdx > 1)
            return;

        DueStepper &motor = _getMotor(motorIdx);

        motor.setFrequency(freq);
        motor.setDirection(direction);
        motor.setStepFraction(resolution);
        motor.setStepTarget(numberSteps);
        motor.start();
    }

    void _startMotorToEncoderPosition(uint8_t index, int32_t target)
    {
        DueStepper &motor       = _getMotor(index);
        MotorId motorId         = (index == 0) ? MotorId::MASK_DRAWER : MotorId::WAFER_DRAWER;
        const uint8_t direction = (target > Encoders::getValue(motorId)) ? HIGH : LOW;

        motor.setDirection(direction);
        motor.setEncoderTarget(target);
        motor.setStepFraction(StepFraction::NONE);
        motor.setFrequency(FREQ_TARGET_MOTOR);
        motor.start();
    }

    void _setConveyorStepperResolution(StepFraction frac, uint32_t *resolutionPins)
    {
        const uint8_t ufrac = static_cast<uint8_t>(frac);

        // `((ufrac & 0x04) | (ufrac & 0x20))` bitwise operation to check if `frac` == 4 || `frac` == 32
        pinMode(resolutionPins[0], ((ufrac & 0x04) | (ufrac & 0x20)) ? INPUT : OUTPUT);

        switch (frac)
        {
        case StepFraction::NONE:
            digitalWrite(resolutionPins[0], LOW);
            digitalWrite(resolutionPins[1], LOW);
            break;
        case StepFraction::FRAC_2:
            digitalWrite(resolutionPins[0], HIGH);
            digitalWrite(resolutionPins[1], LOW);
            break;
        case StepFraction::FRAC_4:
            digitalWrite(resolutionPins[1], LOW);
            break;
        case StepFraction::FRAC_8:
            digitalWrite(resolutionPins[0], LOW);
            digitalWrite(resolutionPins[1], HIGH);
            break;
        case StepFraction::FRAC_16:
            digitalWrite(resolutionPins[0], HIGH);
            digitalWrite(resolutionPins[1], HIGH);
            break;
        case StepFraction::FRAC_32:
            digitalWrite(resolutionPins[1], HIGH);
            break;
        }
    }

    void _sendStopStatus(uint8_t stopIdx)
    {
        if (stopIdx > DRAWERS_STOPS_COUNT) // Invalid stop index
            return;

        const uint8_t buffer[] = {'S', (uint8_t)(stopIdx + COM_STOP_IDX_OFFSET), (uint8_t)(stopStatesDrawer[stopIdx] + '0')};
        Com::send(serial_packet_t(buffer, sizeof(buffer)));
    }

    void _syncStopPinState(uint8_t stopIdx)
    {
        const bool state = !digitalRead(stopPinsDrawer[stopIdx]); // Active low

        if (state != stopStatesDrawer[stopIdx])
        {
            stopStatesDrawer[stopIdx] = state; // Update stored value
            _sendStopStatus(stopIdx);
        }
    }

    uint8_t _getStopPin(uint8_t stopIdx)
    {
        return !digitalRead(stopPinsDrawer[stopIdx]);
    }

    void _checkAllStopPins(void)
    {
        for (uint8_t index = 0; index < DRAWERS_STOPS_COUNT; ++index)
            _syncStopPinState(index);
    }

    void _sendMotorStoppedMsg(uint8_t motorIdx, MotorStopReason reason)
    {
        const uint8_t buffer[] = {'S', 'S', (uint8_t)(motorIdx + COM_MOTOR_IDX_OFFSET), reason};
        Com::send(serial_packet_t(buffer, sizeof(buffer)));
    }

    void _onMaskMotorStopped(MotorStopReason reason)
    {
        _sendMotorStoppedMsg(0, reason);
    }

    void _onWaferMotorStopped(MotorStopReason reason)
    {
        _sendMotorStoppedMsg(1, reason);
    }

    // IRQ callbacks

    void stopISRHandlerCM0(void)
    {
        _syncStopPinState(0u);

        if (stopStatesDrawer[0] &&        // CM0 active
            maskMotor.running() &&        // Motor running
            maskMotor.direction() == LOW) // TODO check dir value: Motor direction is towards CM0 stop
        {
            Drawer::stopMotor(0, MotorStopReason::LimitStopReached);
        }
    }

    void stopISRHandlerCM1(void)
    {
        _syncStopPinState(1u);
    }

    void stopISRHandlerCM2(void)
    {
        _syncStopPinState(2u);
    }

    void stopISRHandlerCM3(void)
    {
        _syncStopPinState(3u);

        if (stopStatesDrawer[3] &&         // CM0 active
            maskMotor.running() &&         // Motor running
            maskMotor.direction() == HIGH) // TODO check dir value: Motor direction is towards CM3 stop
        {
            Drawer::stopMotor(0, MotorStopReason::LimitStopReached);
        }
    }

    void stopISRHandlerCW0(void)
    {
        _syncStopPinState(4u);

        if (stopStatesDrawer[4] &&         // CW0 active
            waferMotor.running() &&        // Motor running
            waferMotor.direction() == LOW) // TODO check dir value: Motor direction is towards CW0 stop
        {
            Drawer::stopMotor(1, MotorStopReason::LimitStopReached);
        }
    }

    void stopISRHandlerCW1(void)
    {
        _syncStopPinState(5u);
    }

    void stopISRHandlerCW2(void)
    {
        _syncStopPinState(6u);

        if (stopStatesDrawer[6] &&          // CW2 active
            waferMotor.running() &&         // Motor running
            waferMotor.direction() == HIGH) // TODO check dir value: Motor direction is towards CW2 stop
        {
            Drawer::stopMotor(1, MotorStopReason::LimitStopReached);
        }
    }
}

namespace Drawer
{
    void setup(void)
    {
        for (int index = 0; index < DRAWERS_STOPS_COUNT; ++index)
            pinMode(stopPinsDrawer[index], INPUT); // Set all stop pins to input mode

        // Interruptions to handle stop state changes
        attachInterrupt(digitalPinToInterrupt(BMCM0), &stopISRHandlerCM0, CHANGE);
        attachInterrupt(digitalPinToInterrupt(BMCM1), &stopISRHandlerCM1, CHANGE);
        attachInterrupt(digitalPinToInterrupt(BMCM2), &stopISRHandlerCM2, CHANGE);
        attachInterrupt(digitalPinToInterrupt(BMCM3), &stopISRHandlerCM3, CHANGE);
        attachInterrupt(digitalPinToInterrupt(BMCW0), &stopISRHandlerCW0, CHANGE);
        attachInterrupt(digitalPinToInterrupt(BMCW1), &stopISRHandlerCW1, CHANGE);
        attachInterrupt(digitalPinToInterrupt(BMCW2), &stopISRHandlerCW2, CHANGE);

        // Mask motor
        maskMotor.setup();
        maskMotor.setTimerNVICPriority(STEPPER_NVIC_PRIORITY);
        maskMotor.attachEncoder(Encoders::getCountPtr(MotorId::MASK_DRAWER));
        maskMotor.attachOnStopCallback(&_onMaskMotorStopped);

        // Wafer motor
        waferMotor.setup();
        waferMotor.setTimerNVICPriority(STEPPER_NVIC_PRIORITY);
        waferMotor.attachEncoder(Encoders::getCountPtr(MotorId::WAFER_DRAWER));
        maskMotor.attachOnStopCallback(&_onWaferMotorStopped);

        // Sync current stops values
        _checkAllStopPins();
        sendAllStopsStatus();
    }

    void loop(void)
    {
        static uint32_t prevStopCheckTimestamp = 0;
        const uint32_t now                     = millis();

        if (now - prevStopCheckTimestamp >= 50)
        {
            // Safety net: stop motors if stop pin interrupt failed to do so

            // Mask
            if (
                maskMotor.running() &&
                ((maskMotor.direction() == LOW && _getStopPin(INDEX_CM0)) ||
                 (maskMotor.direction() == HIGH && _getStopPin(INDEX_CM3))))
            {
                stopMotor(0, MotorStopReason::LimitStopReached);
            }

            // Wafer
            if (
                waferMotor.running() &&
                ((waferMotor.direction() == LOW && _getStopPin(INDEX_CW0)) ||
                 (waferMotor.direction() == HIGH && _getStopPin(INDEX_CW2))))
            {
                stopMotor(1, MotorStopReason::LimitStopReached);
            }

            prevStopCheckTimestamp = now;
        }
    }

    // "CONTROLLERS"

    void startMotor(char *buff, int count)
    {
        if (count < 6)
            return;

        uint8_t motorIdx    = buff[1] - INDEX_ASCII_MOTOR_TPM;
        uint8_t direction   = buff[2] == '1' ? HIGH : LOW;
        uint8_t resolution  = buff[3];
        uint32_t freq       = (uint32_t(buff[4]) << 8) | buff[5];
        uint32_t stepsCount = UINT32_MAX; // default value, unsigned int maximum value

        if (count >= 10)
            stepsCount = (uint32_t((uint8_t)buff[6]) << 24) |
                         (uint32_t((uint8_t)buff[7]) << 16) |
                         (uint32_t((uint8_t)buff[8]) << 8) |
                         (uint8_t)buff[9];

        _startMotor(motorIdx, freq, direction, resolution, stepsCount);
    }

    void moveToEncoderPosition(char *buff, int count)
    {
        if (count < 4)
            return;

        const uint8_t motorIndex = buff[1] - INDEX_ASCII_MOTOR_TPM;

        if (motorIndex > 1)
            return; // Invalid motorIndex

        int32_t targetPosition = 0;
        const bool isNegative  = (buff[2] == '-');
        uint8_t startIdx       = isNegative ? 3 : 2;

        for (uint8_t idx = startIdx; idx < count && buff[idx] != '#'; ++idx)
            targetPosition = (targetPosition * 10) + (buff[idx] - '0');
        if (isNegative)
            targetPosition = -targetPosition;

        _startMotorToEncoderPosition(motorIndex, targetPosition);
    }

    void stopMotor(uint8_t idx, MotorStopReason reason)
    {
        if (idx > 1) // Invalid index
            return;

        DueStepper &motor = _getMotor(idx);

        motor.stop(reason);
    }

    void sendAllStopsStatus(void)
    {
        for (uint8_t idx = 0; idx < DRAWERS_STOPS_COUNT; ++idx)
            _sendStopStatus(idx);
    }
}
