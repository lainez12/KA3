#include <Arduino.h>

#include "SerialTXHandler.h"
#include "autolevel.h"
#include "encoder.h"
#include "pins.h"
#include "timerPriorities.h"

#define AUTOLEVEL_MOTORS_COUNT   3
#define AUTOLEVEL_STOPS_COUNT    6
#define MASKING_ZONE_STOPS_COUNT 3
#define FREQ_TARGET_MOTOR        1000
// Communications related
#define INDEX_ASCII_FIRST_MOTOR_AL '1'
#define COM_MOTOR_IDX_OFFSET       1

const uint32_t maskingZoneStopsPins[MASKING_ZONE_STOPS_COUNT] = {
    LIMITE_ZONE1,
    LIMITE_ZONE2,
    LIMITE_ZONE3,
};
volatile bool maskingZoneStopsStates[MASKING_ZONE_STOPS_COUNT] = {true, true, false};

const uint32_t autolevelStopsPins[AUTOLEVEL_STOPS_COUNT] = {
    BPALG,
    BNALG,
    BPALD,
    BNALD,
    BPALA,
    BNALA,
};
volatile bool autolevelStopsStates[AUTOLEVEL_STOPS_COUNT] = {false, false, false, false, false, false};

namespace
{
    void _setAutolevelStepperResolution(StepFraction frac, uint32_t *resolutionPins);

    DueStepper leftZMotor(
        3, // Timer id
        due_stepper_pins_t{
            .enable           = MALG_nEn,
            .step             = MALG_STEP,
            .direction        = MALG_DIR,
            .resolution       = (uint32_t[3]){MALG_M0, MALG_M1, MALG_M2},
            .resolutionSetter = &_setAutolevelStepperResolution,
        });

    DueStepper rightZMotor(
        4, // Timer id
        due_stepper_pins_t{
            .enable           = MALD_nEn,
            .step             = MALD_STEP,
            .direction        = MALD_DIR,
            .resolution       = (uint32_t[3]){MALD_M0, MALD_M1, MALD_M2},
            .resolutionSetter = &_setAutolevelStepperResolution,
        });

    DueStepper backZMotor(
        5, // Timer id
        due_stepper_pins_t{
            .enable           = MALA_nEn,
            .step             = MALA_STEP,
            .direction        = MALA_DIR,
            .resolution       = (uint32_t[3]){MALA_M0, MALA_M1, MALA_M2},
            .resolutionSetter = &_setAutolevelStepperResolution,
        });

    DueStepper &_getMotor(uint8_t index)
    {
        index = index % AUTOLEVEL_MOTORS_COUNT;

        switch (index)
        {
        case 0:
            return leftZMotor;
        case 1:
            return rightZMotor;
        case 2:
            return backZMotor;
        default:
            break;
        }
        return leftZMotor;
    }

    void _sendAutolevelStopStatus(uint8_t stopIdx)
    {
        if (stopIdx > AUTOLEVEL_STOPS_COUNT - 1) // Invalid stop index
            return;

        const uint8_t buffer[] = {'S', stopIdx, (uint8_t)(autolevelStopsStates[stopIdx] + '0')};
        Com::send(serial_packet_t(buffer, sizeof(buffer)));
    }

    void _syncAutolevelStopPinState(uint8_t stopIdx)
    {
        bool state = digitalRead(autolevelStopsPins[stopIdx]);

        if (stopIdx % 2 == 0)
            state = !state;

        if (state != autolevelStopsStates[stopIdx])
        {
            autolevelStopsStates[stopIdx] = state;
            _sendAutolevelStopStatus(stopIdx);
        }
    }

    void _sendMaskingZoneStopStatus(uint8_t stopIdx)
    {
        if (stopIdx > MASKING_ZONE_STOPS_COUNT - 1) // Invalid stop index
            return;

        const uint8_t buffer[] = {'Z', (uint8_t)(stopIdx + '1'), (uint8_t)(maskingZoneStopsStates[stopIdx] + '0')};
        Com::send(serial_packet_t(buffer, sizeof(buffer)));
    }

    void _syncMaskingZoneStopPinState(uint8_t stopIdx)
    {
        if (stopIdx > 2)
            return; // Invalid index

        const bool rawState     = digitalRead(maskingZoneStopsPins[stopIdx]);
        const bool logicalState = (stopIdx == 2) ? !rawState : rawState;

        if (logicalState != maskingZoneStopsStates[stopIdx])
        {
            maskingZoneStopsStates[stopIdx] = logicalState;
            _sendMaskingZoneStopStatus(stopIdx);
        }
    }

    void _startMotor(uint8_t motorId, uint32_t freq, uint8_t direction, uint8_t resolution, int32_t numSteps)
    {
        if (motorId > 2)
            return;

        DueStepper &motor = _getMotor(motorId);

        motor.setFrequency(freq);
        motor.setDirection(direction);
        motor.setStepFraction(resolution);
        motor.setStepTarget(numSteps);
        motor.start();
    }

    void _startMotorToEncoderPosition(uint8_t index, int32_t target)
    {
        DueStepper &motor = _getMotor(index);
        MotorId motorId;

        if (index == 0)
            motorId = MotorId::Z_LEFT;
        else if (index == 1)
            motorId = MotorId::Z_RIGHT;
        else
            motorId = MotorId::Z_BACK;

        const uint8_t direction = (target > Encoders::getValue(motorId)) ? HIGH : LOW;

        motor.setDirection(direction);
        motor.setEncoderTarget(target);
        motor.setStepFraction(StepFraction::NONE);
        motor.setFrequency(FREQ_TARGET_MOTOR);
        motor.start();
    }

    void _setAutolevelStepperResolution(StepFraction frac, uint32_t *resolutionPins)
    {
        switch (frac)
        {
        case StepFraction::FRAC_16:
            digitalWrite(resolutionPins[0], HIGH);
            digitalWrite(resolutionPins[1], HIGH);
            digitalWrite(resolutionPins[2], HIGH);
            break;
        case StepFraction::FRAC_8:
            digitalWrite(resolutionPins[0], HIGH);
            digitalWrite(resolutionPins[1], HIGH);
            digitalWrite(resolutionPins[2], LOW);
            break;
        case StepFraction::FRAC_4:
            digitalWrite(resolutionPins[0], LOW);
            digitalWrite(resolutionPins[1], HIGH);
            digitalWrite(resolutionPins[2], LOW);
            break;
        case StepFraction::FRAC_2:
            digitalWrite(resolutionPins[0], HIGH);
            digitalWrite(resolutionPins[1], LOW);
            digitalWrite(resolutionPins[2], LOW);
            break;
        case StepFraction::NONE:
            digitalWrite(resolutionPins[0], LOW);
            digitalWrite(resolutionPins[1], LOW);
            digitalWrite(resolutionPins[2], LOW);
            break;
        default:
            break;
        }
    }

    void _sendMotorStoppedMsg(uint8_t motorIdx, MotorStopReason reason)
    {
        const uint8_t buffer[] = {'S', 'S', (uint8_t)(motorIdx + COM_MOTOR_IDX_OFFSET), reason};
        Com::send(serial_packet_t(buffer, sizeof(buffer)));
    }

    void _onLeftZMotorStopped(MotorStopReason reason)
    {
        _sendMotorStoppedMsg(0, reason);
    }

    void _onRightZMotorStopped(MotorStopReason reason)
    {
        _sendMotorStoppedMsg(1, reason);
    }

    void _onBackZMotorStopped(MotorStopReason reason)
    {
        _sendMotorStoppedMsg(2, reason);
    }

    // IRQ callbacks

    void _handleAutolevelStopStateChange(uint8_t stopIdx)
    {
        // Invalid index or stop state changed but not active
        if (stopIdx > (AUTOLEVEL_STOPS_COUNT - 1) || !autolevelStopsStates[stopIdx])
            return;

        const uint8_t direction = ((stopIdx % 2) == 0) ? HIGH : LOW;
        const uint8_t motorIdx  = stopIdx / 2; // Integer division floors the value
        DueStepper &motor       = _getMotor(motorIdx);

        // Motor running AND going towards stop => stop motor (stop active check is above)
        if (motor.running() && motor.direction() == direction)
        {
            Autolevel::stopMotor(motorIdx, MotorStopReason::LimitStopReached);
        }
    }

    void stopISRHandlerLeftT2MKHigh(void)
    {
        _syncAutolevelStopPinState(0u);
        _handleAutolevelStopStateChange(0u);
    }

    void stopISRHandlerLeftT2MKLow(void)
    {
        _syncAutolevelStopPinState(1u);
        _handleAutolevelStopStateChange(1u);
    }

    void stopISRHandlerRightT2MKHigh(void)
    {
        _syncAutolevelStopPinState(2u);
        _handleAutolevelStopStateChange(2u);
    }

    void stopISRHandlerRightT2MKLow(void)
    {
        _syncAutolevelStopPinState(3u);
        _handleAutolevelStopStateChange(3u);
    }

    void stopISRHandlerBackT2MKHigh(void)
    {
        _syncAutolevelStopPinState(4u);
        _handleAutolevelStopStateChange(4u);
    }

    void stopISRHandlerBackT2MKLow(void)
    {
        _syncAutolevelStopPinState(5u);
        _handleAutolevelStopStateChange(5u);
    }

}

namespace Autolevel
{
    void setup(void)
    {
        // Set T2MK Low/High pin as input
        for (int index = 0; index < AUTOLEVEL_STOPS_COUNT; ++index)
            pinMode(autolevelStopsPins[index], INPUT);
        // Set Z1, Z2, Wafer ON pin as input
        for (int index = 0; index < MASKING_ZONE_STOPS_COUNT; ++index)
            pinMode(maskingZoneStopsPins[index], INPUT);

        attachInterrupt(digitalPinToInterrupt(BNALG), &stopISRHandlerLeftT2MKLow, CHANGE);
        attachInterrupt(digitalPinToInterrupt(BPALG), &stopISRHandlerLeftT2MKHigh, CHANGE);
        attachInterrupt(digitalPinToInterrupt(BNALD), &stopISRHandlerRightT2MKLow, CHANGE);
        attachInterrupt(digitalPinToInterrupt(BPALD), &stopISRHandlerRightT2MKHigh, CHANGE);
        attachInterrupt(digitalPinToInterrupt(BNALA), &stopISRHandlerBackT2MKLow, CHANGE);
        attachInterrupt(digitalPinToInterrupt(BPALA), &stopISRHandlerBackT2MKHigh, CHANGE);

        // Left Z
        leftZMotor.setup();
        leftZMotor.setTimerNVICPriority(STEPPER_NVIC_PRIORITY);
        leftZMotor.attachEncoder(Encoders::getCountPtr(MotorId::Z_LEFT));
        leftZMotor.attachOnStopCallback(&_onLeftZMotorStopped);

        // Right Z
        rightZMotor.setup();
        rightZMotor.setTimerNVICPriority(STEPPER_NVIC_PRIORITY);
        rightZMotor.attachEncoder(Encoders::getCountPtr(MotorId::Z_RIGHT));
        rightZMotor.attachOnStopCallback(&_onRightZMotorStopped);

        // Back Z
        backZMotor.setup();
        backZMotor.setTimerNVICPriority(STEPPER_NVIC_PRIORITY);
        backZMotor.attachEncoder(Encoders::getCountPtr(MotorId::Z_BACK));
        backZMotor.attachOnStopCallback(&_onBackZMotorStopped);

        // Update all autolevel stop pins states
        checkAllAutolevelStopPins();
        sendAllAutolevelStopsStatus();
        // Update all masking zone stop pins states
        checkAllMaskingZoneStopPins();
        sendAllMaskingZoneStopsStatus();
    }

    void loop(void)
    {
        static uint32_t prevStopCheckTimestamp = 0;
        const uint32_t now                     = millis();

        if (now - prevStopCheckTimestamp >= 50)
        {
            // Safety net: stop motors if stop pin interrupt failed to do so
            if (leftZMotor.running())
            {
                _handleAutolevelStopStateChange(0u);
                _handleAutolevelStopStateChange(1u);
            }
            if (rightZMotor.running())
            {
                _handleAutolevelStopStateChange(2u);
                _handleAutolevelStopStateChange(3u);
            }
            if (backZMotor.running())
            {
                _handleAutolevelStopStateChange(4u);
                _handleAutolevelStopStateChange(5u);
            }
            prevStopCheckTimestamp = now; // Update last check timestamp
        }
    }

    void checkAllAutolevelStopPins(void)
    {
        for (uint8_t idx = 0; idx < AUTOLEVEL_STOPS_COUNT; ++idx)
            _syncAutolevelStopPinState(idx);
    }

    void checkAllMaskingZoneStopPins(void)
    {
        for (uint8_t idx = 0; idx < MASKING_ZONE_STOPS_COUNT; ++idx)
            _syncMaskingZoneStopPinState(idx);
    }

    void startMotor(char *buff, int count)
    {
        if (count < 6)
            return;

        uint8_t motorId    = buff[1] - INDEX_ASCII_FIRST_MOTOR_AL;
        uint8_t direction  = buff[2] == '1' ? HIGH : LOW;
        uint8_t resolution = buff[3];
        uint32_t freq      = (uint32_t(buff[4]) << 8) + buff[5];
        int32_t numberStep = -1;

        if (count >= 10)
            numberStep = (uint32_t((uint8_t)buff[6]) << 24) |
                         (uint32_t((uint8_t)buff[7]) << 16) |
                         (uint32_t((uint8_t)buff[8]) << 8) |
                         (uint8_t)buff[9];

        _startMotor(motorId, freq, direction, resolution, numberStep);
    }

    void moveToEncoderPosition(char *buff, int count)
    {
        if (count < 4)
            return;

        const uint8_t motorIndex = buff[1] - INDEX_ASCII_FIRST_MOTOR_AL;

        if (motorIndex > 2)
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

    void stopMotor(uint8_t index, MotorStopReason reason)
    {
        if (index > AUTOLEVEL_MOTORS_COUNT - 1)
            return;

        DueStepper &motor = _getMotor(index);

        motor.stop(reason);
    }

    void sendAllAutolevelStopsStatus(void)
    {
        for (uint8_t idx = 0; idx < AUTOLEVEL_STOPS_COUNT; ++idx)
            _sendAutolevelStopStatus(idx);
    }

    void sendAllMaskingZoneStopsStatus(void)
    {
        for (uint8_t idx = 0; idx < MASKING_ZONE_STOPS_COUNT; ++idx)
            _sendMaskingZoneStopStatus(idx);
    }
}
