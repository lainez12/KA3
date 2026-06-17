#include "motors.h"
#include "DueStepper.hpp"
#include "SerialTXHandler.h"
#include "definitions.h"
#include "encoder.h"
#include "pins.h"

// -----------------------------------------------------------------------------
// Constants & Mappings
// -----------------------------------------------------------------------------

#define FREQ_TARGET_MOTOR 1000

// We use 255 to flag a limit pin that doesn't hard-stop a motor (e.g., intermediate sensors or zones)
#define NO_MOTOR 255

// Defines a data-driven mapping for limit switches.
// Instead of writing explicit conditional logic for each switch inside the ISRs,
// this struct routes a triggered pin directly to its associated DueStepper instance.
struct LimitMapping {
    uint8_t motorIdx;
    uint8_t stopDirection;
};

namespace Motors
{
    namespace Internal
    {
        volatile bool alignmentEnabled = true;

        // -------------------------------------------------------------------------
        // Unified Hardware Limit Switches & Elevator Zone Limits
        // -------------------------------------------------------------------------

        const uint32_t limitPins[LIMIT_PINS_COUNT] = {
            Z_LEFT_HI_LIMIT_PIN, Z_LEFT_LO_LIMIT_PIN, Z_RIGHT_HI_LIMIT_PIN, Z_RIGHT_LO_LIMIT_PIN, Z_BACK_HI_LIMIT_PIN, Z_BACK_LO_LIMIT_PIN,                                            // 0-5: Autolevel Extreme Limits
            MASK_CONV_LIMIT_CM0_PIN, MASK_CONV_LIMIT_CM1_PIN, MASK_CONV_LIMIT_CM2_PIN, MASK_CONV_LIMIT_CM3_PIN, // 6-9: Mask Drawer Limits
            WAFER_CONV_LIMIT_CW0_PIN, WAFER_CONV_LIMIT_CW1_PIN, WAFER_CONV_LIMIT_CW2_PIN,                       // 10-12: Wafer Drawer Limits
            Z1_LIMIT, Z2_LIMIT, WAFER_ON_LIMIT                                                                  // 13-15: Elevator Zone Limits
        };

        volatile bool limitStates[LIMIT_PINS_COUNT] = {false};

        const LimitMapping limitMaps[LIMIT_PINS_COUNT] = {
            {Z_LEFT, HIGH},       // 0: Z_LEFT_HI_LIMIT_PIN (Left Z High Limit)
            {Z_LEFT, LOW},        // 1: Z_LEFT_LO_LIMIT_PIN (Left Z Low Limit)
            {Z_RIGHT, HIGH},      // 2: Z_RIGHT_HI_LIMIT_PIN (Right Z High Limit)
            {Z_RIGHT, LOW},       // 3: Z_RIGHT_LO_LIMIT_PIN (Right Z Low Limit)
            {Z_BACK, HIGH},       // 4: Z_BACK_HI_LIMIT_PIN (Back Z High Limit)
            {Z_BACK, LOW},        // 5: Z_BACK_LO_LIMIT_PIN (Back Z Low Limit)
            {MASK_DRAWER, LOW},   // 6: CM0 -> Mask LOW
            {NO_MOTOR, 0},        // 7: CM1 -> Just reports state
            {NO_MOTOR, 0},        // 8: CM2 -> Just reports state
            {MASK_DRAWER, HIGH},  // 9: CM3 -> Mask HIGH
            {WAFER_DRAWER, LOW},  // 10: CW0 -> Wafer LOW
            {NO_MOTOR, 0},        // 11: CW1 -> Just reports state
            {WAFER_DRAWER, HIGH}, // 12: CW2 -> Wafer HIGH
            {NO_MOTOR, 0},        // 13: Z1 -> Just reports state
            {NO_MOTOR, 0},        // 14: Z2 -> Just reports state
            {NO_MOTOR, 0}         // 15: W_ON -> Just reports state
        };

        // -------------------------------------------------------------------------
        // Stepper Control & Array
        // -------------------------------------------------------------------------

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

        void _setConveyorStepperResolution(StepFraction frac, uint32_t *resolutionPins)
        {
            const uint8_t ufrac = static_cast<uint8_t>(frac);

            // TRICKY PART (High-Z/floating State): Certain stepper drivers use tri-state logic on resolution pins.
            // By setting the pin to INPUT, we put it in a High-Z (floating) state required to achieve specific
            // micro-stepping fractions (e.g. 1/4).
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

        DueStepper steppers[MOTORS_COUNT] = {
            DueStepper(
                3,
                {.enable           = Z_LEFT_EN_PIN,
                 .step             = Z_LEFT_STEP_PIN,
                 .direction        = Z_LEFT_DIR_PIN,
                 .resolution       = (uint32_t[3]){Z_LEFT_RES_M0_PIN, Z_LEFT_RES_M1_PIN, Z_LEFT_RES_M2_PIN},
                 .resolutionSetter = &_setAutolevelStepperResolution}),
            DueStepper(
                4,
                {.enable           = Z_RIGHT_EN_PIN,
                 .step             = Z_RIGHT_STEP_PIN,
                 .direction        = Z_RIGHT_DIR_PIN,
                 .resolution       = (uint32_t[3]){Z_RIGHT_RES_M0_PIN, Z_RIGHT_RES_M1_PIN, Z_RIGHT_RES_M2_PIN},
                 .resolutionSetter = &_setAutolevelStepperResolution}),
            DueStepper(
                5,
                {.enable           = Z_BACK_EN_PIN,
                 .step             = Z_BACK_STEP_PIN,
                 .direction        = Z_BACK_DIR_PIN,
                 .resolution       = (uint32_t[3]){Z_BACK_RES_M0_PIN, Z_BACK_RES_M1_PIN, Z_BACK_RES_M2_PIN},
                 .resolutionSetter = &_setAutolevelStepperResolution}),
            DueStepper(
                1,
                {.enable           = MASK_CONV_EN_PIN,
                 .step             = MASK_CONV_STEP_PIN,
                 .direction        = MASK_CONV_DIR_PIN,
                 .resolution       = (uint32_t[2]){MASK_CONV_RES_M0_PIN, MASK_CONV_RES_M1_PIN},
                 .resolutionSetter = &_setConveyorStepperResolution}),
            DueStepper(
                2,
                {.enable           = WAFER_CONV_EN_PIN,
                 .step             = WAFER_CONV_STEP_PIN,
                 .direction        = WAFER_CONV_DIR_PIN,
                 .resolution       = (uint32_t[2]){WAFER_CONV_RES_M0_PIN, WAFER_CONV_RES_M1_PIN},
                 .resolutionSetter = &_setConveyorStepperResolution})};

        // -------------------------------------------------------------------------
        // State Senders & Synchronizers
        // -------------------------------------------------------------------------

        /**
         * @brief Normalizes raw hardware logic levels into logical TRUE (active) / FALSE (inactive)
         */
        bool _getLimitPinLogicalState(uint8_t limitIdx)
        {
            bool rawState = digitalRead(limitPins[limitIdx]);

            // Hardware Inversions Map:
            // Normalize varying NC/NO electrical wirings so the software ALWAYS sees TRUE = Collision (Active limit)
            // - Autolevel High limits (0, 2, 4) are Active Low
            // - All Drawer limits (6 to 12) are Active Low
            // - Wafer ON (15) is Active Low
            if (limitIdx == 0 || limitIdx == 2 || limitIdx == 4 ||
                (limitIdx >= 6 && limitIdx <= 12) || limitIdx == 15)
            {
                return !rawState;
            }

            return rawState;
        }

        /**
         * @brief Sends the state of a limit switch. Automatically formats the serial
         * payload depending on whether it's a Motor limit ('S') or a Zone limit ('Z').
         */
        void _sendLimitStatus(uint8_t limitIdx)
        {
            if (limitIdx >= LIMIT_PINS_COUNT)
                return;

            if (limitIdx < STANDARD_LIMITS_COUNT)
            {
                // Motor limit -> payload: 'S', limitIdx, '0' or '1'
                const uint8_t buffer[] = {'S', limitIdx, (uint8_t)(limitStates[limitIdx] + '0')};
                Com::send(serial_packet_t(buffer, sizeof(buffer)));
            }
            else
            {
                // Zone limit -> payload: 'Z', '1'-'3', '0' or '1'
                const uint8_t stopIdx  = limitIdx - STANDARD_LIMITS_COUNT;
                const uint8_t buffer[] = {'Z', (uint8_t)(stopIdx + '1'), (uint8_t)(limitStates[limitIdx] + '0')};
                Com::send(serial_packet_t(buffer, sizeof(buffer)));
            }
        }

        void _syncLimitPinState(uint8_t limitIdx)
        {
            bool state = _getLimitPinLogicalState(limitIdx);

            if (state != limitStates[limitIdx])
            {
                limitStates[limitIdx] = state;
                _sendLimitStatus(limitIdx);
            }
        }

        // -------------------------------------------------------------------------
        // Safety & Callbacks
        // -------------------------------------------------------------------------

        /**
         * @brief Checks if a movement is currently blocked by an active limit switch
         */
        bool _isMovementBlocked(uint8_t motorIdx, uint8_t direction)
        {
            for (uint8_t i = 0; i < LIMIT_PINS_COUNT; ++i)
            {
                if (limitMaps[i].motorIdx == motorIdx &&
                    limitMaps[i].stopDirection == direction &&
                    limitStates[i] == true)
                {
                    return true;
                }
            }
            return false;
        }

        void _sendMotorStoppedMsg(uint8_t motorIdx, MotorStopReason reason)
        {
            // Translates 0-4 back to ASCII '1'-'5'
            const uint8_t buffer[] = {'S', 'S', (uint8_t)(motorIdx + '1'), reason};
            Com::send(serial_packet_t(buffer, sizeof(buffer)));
        }

        // Stepper hardware callbacks
        void _onLeftZMotorStopped(MotorStopReason r)
        {
            _sendMotorStoppedMsg(Z_LEFT, r);
        }
        void _onRightZMotorStopped(MotorStopReason r)
        {
            _sendMotorStoppedMsg(Z_RIGHT, r);
        }
        void _onBackZMotorStopped(MotorStopReason r)
        {
            _sendMotorStoppedMsg(Z_BACK, r);
        }
        void _onMaskMotorStopped(MotorStopReason r)
        {
            _sendMotorStoppedMsg(MASK_DRAWER, r);
        }
        void _onWaferMotorStopped(MotorStopReason r)
        {
            _sendMotorStoppedMsg(WAFER_DRAWER, r);
        }

        /**
         * @brief Generic limit handler called by the 16 limits ISRs.
         * Handles both limits and zones seamlessly.
         */
        void _handleLimitISR(uint8_t limitIdx)
        {
            _syncLimitPinState(limitIdx); // Syncs state and notifies host via UART DMA

            const LimitMapping &map = limitMaps[limitIdx];
            if (map.motorIdx == NO_MOTOR)
                return; // Ignores info-only limits

            DueStepper &motor = steppers[map.motorIdx];

            // Hardware safety constraint: Only stop the motor if the pin is currently active,
            // the motor is moving, AND it's driving INTO the switch.
            if (limitStates[limitIdx] && motor.running() && motor.direction() == map.stopDirection)
            {
                motor.stop(MotorStopReason::LimitStopReached);
            }
        }

        // Descriptive ISR Handlers
        void limitISRHandlerLeftT2MKHigh()
        {
            _handleLimitISR(0);
        }
        void limitISRHandlerLeftT2MKLow()
        {
            _handleLimitISR(1);
        }
        void limitISRHandlerRightT2MKHigh()
        {
            _handleLimitISR(2);
        }
        void limitISRHandlerRightT2MKLow()
        {
            _handleLimitISR(3);
        }
        void limitISRHandlerBackT2MKHigh()
        {
            _handleLimitISR(4);
        }
        void limitISRHandlerBackT2MKLow()
        {
            _handleLimitISR(5);
        }

        void stopISRHandlerCM0()
        {
            _handleLimitISR(6);
        }
        void stopISRHandlerCM1()
        {
            _handleLimitISR(7);
        }
        void stopISRHandlerCM2()
        {
            _handleLimitISR(8);
        }
        void stopISRHandlerCM3()
        {
            _handleLimitISR(9);
        }

        void stopISRHandlerCW0()
        {
            _handleLimitISR(10);
        }
        void stopISRHandlerCW1()
        {
            _handleLimitISR(11);
        }
        void stopISRHandlerCW2()
        {
            _handleLimitISR(12);
        }

        void zoneLimitISRHandlerZone1()
        {
            _handleLimitISR(13);
        }
        void zoneLimitISRHandlerZone2()
        {
            _handleLimitISR(14);
        }
        void zoneLimitISRHandlerZone3()
        {
            _handleLimitISR(15);
        }

        // -------------------------------------------------------------------------
        // DueStepper helper functions
        // -------------------------------------------------------------------------

        void _startMotor(uint8_t motorIdx, uint32_t freq, uint8_t direction, uint8_t resolution, uint32_t numSteps)
        {
            if (_isMovementBlocked(motorIdx, direction))
                return;

            DueStepper &motor = steppers[motorIdx];

            motor.setFrequency(freq);
            motor.setDirection(direction);
            motor.setStepFraction(resolution);
            motor.setStepTarget(numSteps);
            motor.start();
        }

        void _startMotorToEncoderPosition(uint8_t motorIdx, int32_t target)
        {
            const uint8_t direction = (target > Encoders::getValue(static_cast<MotorId>(motorIdx))) ? HIGH : LOW;

            if (_isMovementBlocked(motorIdx, direction))
                return;

            DueStepper &motor = steppers[motorIdx];

            motor.setDirection(direction);
            motor.setEncoderTarget(target);
            motor.setStepFraction(StepFraction::NONE);
            motor.setFrequency(FREQ_TARGET_MOTOR);
            motor.start();
        }
    }
}

namespace Motors
{
    using namespace Internal;

    void setup(void)
    {
        // 1. Set all limit pins as inputs
        for (int i = 0; i < LIMIT_PINS_COUNT; ++i)
            pinMode(limitPins[i], INPUT);

        // 2. Attach Hardware Interrupts to everything (Standard + Zones)
        void (*isrArray[LIMIT_PINS_COUNT])() = {
            &limitISRHandlerLeftT2MKHigh, &limitISRHandlerLeftT2MKLow,
            &limitISRHandlerRightT2MKHigh, &limitISRHandlerRightT2MKLow,
            &limitISRHandlerBackT2MKHigh, &limitISRHandlerBackT2MKLow,
            &stopISRHandlerCM0, &stopISRHandlerCM1, &stopISRHandlerCM2, &stopISRHandlerCM3,
            &stopISRHandlerCW0, &stopISRHandlerCW1, &stopISRHandlerCW2,
            &zoneLimitISRHandlerZone1, &zoneLimitISRHandlerZone2, &zoneLimitISRHandlerZone3};

        for (uint8_t i = 0; i < LIMIT_PINS_COUNT; ++i)
            attachInterrupt(digitalPinToInterrupt(limitPins[i]), isrArray[i], CHANGE);

        // 3. Initialize Steppers
        void (*stopCbArray[MOTORS_COUNT])(MotorStopReason) = {
            &_onLeftZMotorStopped, &_onRightZMotorStopped, &_onBackZMotorStopped,
            &_onMaskMotorStopped, &_onWaferMotorStopped};

        for (uint8_t i = 0; i < MOTORS_COUNT; ++i)
        {
            // Boots up the TC peripherals (Timer/Counters) for zero-CPU-overhead stepping.
            steppers[i].setup();
            steppers[i].setTimerNVICPriority(STEPPER_NVIC_PRIORITY);
            steppers[i].attachEncoder(Encoders::getCountPtr(static_cast<MotorId>(i)));
            steppers[i].attachOnStopCallback(stopCbArray[i]);
        }

        // Sync initial states to host
        checkAllStdLimitPins();
        sendAllStdLimitsStatus();
        checkAllZLimitPins();
        sendAllZLimitsStatus();
    }

    void loop(void)
    {
        static uint32_t prevStopCheckTimestamp = 0;
        const uint32_t now                     = millis();

        // Safety Net (Software Fallback)
        if (now - prevStopCheckTimestamp >= 100)
        {
            // Sync states to catch missed interrupts
            for (uint8_t i = 0; i < LIMIT_PINS_COUNT; ++i)
                _syncLimitPinState(i);

            // Safety net: Force stop if ISR failed
            // Redundantly checks all limit switches. If an EMI glitch or bouncing contact
            // causes the hardware EXTI/ISR to miss an edge, this loop will catch the DC state
            // and safely halt the offending motor.
            for (uint8_t i = 0; i < LIMIT_PINS_COUNT; ++i)
            {
                const LimitMapping &map = limitMaps[i];

                if (map.motorIdx == NO_MOTOR)
                    continue;

                DueStepper &motor = steppers[map.motorIdx];

                if (limitStates[i] && motor.running() && motor.direction() == map.stopDirection)
                {
                    motor.stop(MotorStopReason::LimitStopReached);
                }
            }
            prevStopCheckTimestamp = now;
        }
    }

    // -------------------------------------------------------------------------
    // Commands & Controllers
    // -------------------------------------------------------------------------

    void checkAllStdLimitPins(void)
    {
        for (uint8_t idx = 0; idx < STANDARD_LIMITS_COUNT; ++idx)
            _syncLimitPinState(idx);
    }

    void checkAllZLimitPins(void)
    {
        for (uint8_t idx = STANDARD_LIMITS_COUNT; idx < LIMIT_PINS_COUNT; ++idx)
            _syncLimitPinState(idx);
    }

    void startMotor(char *buff, int count)
    {
        if (count < 6)
            return;

        // Extracts motor index and converts ASCII '1'-'5' to 0-4
        uint8_t motorIdx = buff[1] - '1';
        if (motorIdx >= MOTORS_COUNT)
            return;

        uint8_t direction   = buff[2] == '1' ? HIGH : LOW;
        uint8_t resolution  = buff[3];
        uint32_t freq       = (uint32_t(buff[4]) << 8) | buff[5];
        uint32_t numberStep = UINT32_MAX; // Run indefinitely by default (0xFFFFFFFF)

        // Deserialize 32-bit step target from 4 bytes (Big-Endian format)
        if (count >= 10)
            numberStep = (uint32_t((uint8_t)buff[6]) << 24) |
                         (uint32_t((uint8_t)buff[7]) << 16) |
                         (uint32_t((uint8_t)buff[8]) << 8) |
                         (uint8_t)buff[9];

        _startMotor(motorIdx, freq, direction, resolution, numberStep);
    }

    void moveToEncoderPosition(char *buff, int count)
    {
        if (count < 4)
            return;

        uint8_t motorIdx = buff[1] - '1';
        if (motorIdx >= MOTORS_COUNT)
            return;

        int32_t targetPosition = 0;
        const bool isNegative  = (buff[2] == '-');
        uint8_t startIdx       = isNegative ? 3 : 2;

        for (uint8_t i = startIdx; i < count && buff[i] != '#'; ++i)
            targetPosition = (targetPosition * 10) + (buff[i] - '0');

        if (isNegative)
            targetPosition = -targetPosition;

        _startMotorToEncoderPosition(motorIdx, targetPosition);
    }

    void stopMotorCommand(char *buff, int count)
    {
        if (count > 1)
        {
            uint8_t motorIdx = buff[1] - '1';
            if (motorIdx < MOTORS_COUNT)
                stopMotor(static_cast<MotorId>(motorIdx), MotorStopReason::SoftwareOrder);
        }
    }

    void stopMotor(MotorId index, MotorStopReason reason)
    {
        if (index < MOTORS_COUNT)
            steppers[index].stop(reason);
    }

    void sendAllStdLimitsStatus(void)
    {
        for (uint8_t idx = 0; idx < STANDARD_LIMITS_COUNT; ++idx)
            _sendLimitStatus(idx);
    }

    void sendAllZLimitsStatus(void)
    {
        for (uint8_t idx = STANDARD_LIMITS_COUNT; idx < LIMIT_PINS_COUNT; ++idx)
            _sendLimitStatus(idx);
    }
}
