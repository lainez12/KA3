#include "motors.h"
#include "DueStepper.hpp"
#include "SerialTXHandler.h"
#include "definitions.h"
#include "encoder.h"
#include "pins.h"
#include "utils.h"

// -----------------------------------------------------------------------------
// Constants & Mappings
// -----------------------------------------------------------------------------

#define LEFT_CAM_X_MOTOR_IDX  0
#define LEFT_CAM_Y_MOTOR_IDX  1
#define RIGHT_CAM_X_MOTOR_IDX 2
#define RIGHT_CAM_Y_MOTOR_IDX 3
#define X_STAGE_MOTOR_IDX     4
#define Y_STAGE_MOTOR_IDX     5
#define TH_STAGE_MOTOR_IDX    6

#define LEFT_CAM_X_LEFT_LIMIT_IDX   0
#define LEFT_CAM_Y_FRONT_LIMIT_IDX  1
#define RIGHT_CAM_X_RIGHT_LIMIT_IDX 2
#define RIGHT_CAM_Y_FRONT_LIMIT_IDX 3
#define X_STAGE_LEFT_LIMIT_IDX      4
#define Y_STAGE_FRONT_LIMIT_IDX     5
#define TH_STAGE_CW_LIMIT_IDX       6
#define X_STAGE_RIGHT_LIMIT_IDX     7
#define Y_STAGE_BACK_LIMIT_IDX      8
#define TH_STAGE_CCW_LIMIT_IDX      9
// TODO: add when available
// LEFT_CAM_X_RIGHT
// LEFT_CAM_Y_BACK
// RIGHT_CAM_X_LEFT
// RIGHT_CAM_Y_BACK

/**
 * @brief Maps a limit switch to the specific motor it protects, and the direction
 * of travel that should trigger the stop. This prevents us from needing dozens of
 * "if (motor == X && dir == Y)" statements later.
 */
struct LimitMapping {
    uint8_t motorIdx;
    uint8_t stopDirection; // HIGH or LOW
};

namespace Motors
{
    // The unnamed namespace hides these variables and functions from the rest of the program,
    // acting like the 'static' keyword in C. They can only be used inside motors.cpp
    namespace
    {
        volatile bool alignmentEnabled = true;

        // Mapping of the 10 limit switch pins to their hardware definitions
        const uint8_t limitPins[LIMIT_PINS_COUNT] = {
            LEFT_CAM_X_LIMIT_PIN, LEFT_CAM_Y_LIMIT_PIN, RIGHT_CAM_X_LIMIT_PIN, RIGHT_CAM_Y_LIMIT_PIN, // Cameras limits
            STAGE_X_POS_LIMIT_PIN, STAGE_Y_POS_LIMIT_PIN, STAGE_TH_POS_LIMIT_PIN,                     // Alignment stages "positive" limits
            STAGE_X_NEG_LIMIT_PIN, STAGE_Y_NEG_LIMIT_PIN, STAGE_TH_NEG_LIMIT_PIN                      // Alignment stages "negative" limits (using analog pins)
        };

        // Caches the last known state of each limit switch to detect changes
        volatile bool limitStates[LIMIT_PINS_COUNT] = {false};

        // This array links everything together: "Limit Switch 7 stops Motor 4 when moving HIGH"
        const LimitMapping limitMaps[LIMIT_PINS_COUNT] = {
            {LEFT_CAM_X_MOTOR_IDX, LOW},  // 0: Left Cam X Left
            {LEFT_CAM_Y_MOTOR_IDX, LOW},  // 1: Left Cam Y Front
            {RIGHT_CAM_X_MOTOR_IDX, LOW}, // 2: Right Cam X Right
            {RIGHT_CAM_Y_MOTOR_IDX, LOW}, // 3: Right Cam Y Front
            {X_STAGE_MOTOR_IDX, LOW},     // 4: X Stage Pos (Left)
            {Y_STAGE_MOTOR_IDX, LOW},     // 5: Y Stage Pos (Front)
            {TH_STAGE_MOTOR_IDX, LOW},    // 6: Th Stage Pos (CW)
            {X_STAGE_MOTOR_IDX, HIGH},    // 7: X Stage Neg (Right)
            {Y_STAGE_MOTOR_IDX, HIGH},    // 8: Y Stage Neg (Back)
            {TH_STAGE_MOTOR_IDX, HIGH}    // 9: Th Stage Neg (CCW)
        };

        /**
         * @brief Sets the hardware pins corresponding to the micro-stepping resolution.
         *
         * TRICKY PART:
         * The bitwise logic `(ufrac & 0x04) | (ufrac & 0x20)` checks if the fraction is
         * exactly 4 or 32 to detect if a floating pin is needed.
         */
        void _setStepperResolution(StepFraction frac, uint32_t *resolutionPins)
        {
            const uint8_t ufrac = static_cast<uint8_t>(frac);
            pinMode(resolutionPins[0], ((ufrac & 0x04) | (ufrac & 0x20)) ? INPUT : OUTPUT);
            pinMode(resolutionPins[1], OUTPUT);

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

        // -------------------------------------------------------------------------
        // Motor Array Initialization
        // -------------------------------------------------------------------------

        // This array holds all 7 motor objects.
        // By keeping them in an array, we can loop through them using their ID (0 to 6).
        DueStepper steppers[MOTORS_COUNT] = {
            DueStepper(
                0,
                {.enable           = LEFT_CAM_X_EN_PIN,
                 .step             = LEFT_CAM_X_STEP_PIN,
                 .direction        = LEFT_CAM_X_DIR_PIN,
                 .resolution       = (uint32_t[2]){LEFT_CAM_X_RES_M0_PIN, LEFT_CAM_X_RES_M1_PIN},
                 .resolutionSetter = &_setStepperResolution}),
            DueStepper(
                1,
                {.enable           = LEFT_CAM_Y_EN_PIN,
                 .step             = LEFT_CAM_Y_STEP_PIN,
                 .direction        = LEFT_CAM_Y_DIR_PIN,
                 .resolution       = (uint32_t[2]){LEFT_CAM_Y_RES_M0_PIN, LEFT_CAM_Y_RES_M1_PIN},
                 .resolutionSetter = &_setStepperResolution}),
            DueStepper(
                2,
                {.enable           = RIGHT_CAM_X_EN_PIN,
                 .step             = RIGHT_CAM_X_STEP_PIN,
                 .direction        = RIGHT_CAM_X_DIR_PIN,
                 .resolution       = (uint32_t[2]){RIGHT_CAM_X_RES_M0_PIN, RIGHT_CAM_X_RES_M1_PIN},
                 .resolutionSetter = &_setStepperResolution}),
            DueStepper(
                3,
                {.enable           = RIGHT_CAM_Y_EN_PIN,
                 .step             = RIGHT_CAM_Y_STEP_PIN,
                 .direction        = RIGHT_CAM_Y_DIR_PIN,
                 .resolution       = (uint32_t[2]){RIGHT_CAM_Y_RES_M0_PIN, RIGHT_CAM_Y_RES_M1_PIN},
                 .resolutionSetter = &_setStepperResolution}),
            DueStepper(
                4,
                {.enable           = STAGE_X_EN_PIN,
                 .step             = STAGE_X_STEP_PIN,
                 .direction        = STAGE_X_DIR_PIN,
                 .resolution       = (uint32_t[2]){STAGE_X_RES_M0_PIN, STAGE_X_RES_M1_PIN},
                 .resolutionSetter = &_setStepperResolution}),
            DueStepper(
                5,
                {.enable           = STAGE_Y_EN_PIN,
                 .step             = STAGE_Y_STEP_PIN,
                 .direction        = STAGE_Y_DIR_PIN,
                 .resolution       = (uint32_t[2]){STAGE_Y_RES_M0_PIN, STAGE_Y_RES_M1_PIN},
                 .resolutionSetter = &_setStepperResolution}),
            DueStepper(
                6,
                {.enable           = STAGE_TH_EN_PIN,
                 .step             = STAGE_TH_STEP_PIN,
                 .direction        = STAGE_TH_DIR_PIN,
                 .resolution       = (uint32_t[2]){STAGE_TH_RES_M0_PIN, STAGE_TH_RES_M1_PIN},
                 .resolutionSetter = &_setStepperResolution})};

        // -------------------------------------------------------------------------
        // Callbacks & ISR logic
        // -------------------------------------------------------------------------

        /**
         * @brief Sends a serial message indicating a motor has stopped, and why.
         */
        void _sendMotorStoppedMsg(uint8_t motorIdx, MotorStopReason reason)
        {
            uint8_t asciiCode = KUtils::motorIndexToAsciiByte(motorIdx);

            if (asciiCode == INVALID_MOTOR_INDEX)
                return;

            const uint8_t buffer[] = {'S', 'S', asciiCode, (uint8_t)reason};
            Com::send(serial_packet_t(buffer, sizeof(buffer)));
        }

        // Stepper hardware callbacks: Triggered automatically when the DueStepper library finishes a move.
        void _onLeftCamXStopped(MotorStopReason r)
        {
            _sendMotorStoppedMsg(LEFT_CAM_X_MOTOR_IDX, r);
        }
        void _onLeftCamYStopped(MotorStopReason r)
        {
            _sendMotorStoppedMsg(LEFT_CAM_Y_MOTOR_IDX, r);
        }
        void _onRightCamXStopped(MotorStopReason r)
        {
            _sendMotorStoppedMsg(RIGHT_CAM_X_MOTOR_IDX, r);
        }
        void _onRightCamYStopped(MotorStopReason r)
        {
            _sendMotorStoppedMsg(RIGHT_CAM_Y_MOTOR_IDX, r);
        }
        void _onXStageStopped(MotorStopReason r)
        {
            _sendMotorStoppedMsg(X_STAGE_MOTOR_IDX, r);
        }
        void _onYStageStopped(MotorStopReason r)
        {
            _sendMotorStoppedMsg(Y_STAGE_MOTOR_IDX, r);
        }
        void _onThStageStopped(MotorStopReason r)
        {
            _sendMotorStoppedMsg(TH_STAGE_MOTOR_IDX, r);
        }

        /**
         * @brief Builds and sends the serial message reporting the state of a limit switch.
         */
        void _sendLimitStatus(uint8_t limitIdx)
        {
            if (limitIdx >= LIMIT_PINS_COUNT) // Invalid limit index
                return;

            uint8_t code = KUtils::limitIndexToByteCode(limitIdx);

            if (code != INVALID_LIMIT_INDEX)
                return;

            const uint8_t limitValueByte = (uint8_t)(limitStates[limitIdx] + '0');
            const uint8_t buffer[]       = {'S', code, limitValueByte};

            Com::send(serial_packet_t(buffer, sizeof(buffer)));
        }

        /**
         * @brief Reads the physical pin state of a limit switch.
         */
        uint8_t _getLimitPin(uint8_t limitIdx)
        {
            return digitalRead(limitPins[limitIdx]); // Active low ?
        }

        /**
         * @brief Checks if a limit switch has changed state since we last checked.
         * If it has changed, it updates the cached state and notifies the software.
         */
        void _syncLimitPinState(uint8_t limitIdx)
        {
            const bool state = _getLimitPin(limitIdx);

            if (state != limitStates[limitIdx])
            {
                limitStates[limitIdx] = state; // Update stored value
                uint8_t code          = KUtils::limitIndexToByteCode(limitIdx);

                if (code != INVALID_LIMIT_INDEX)
                    _sendLimitStatus(limitIdx);
            }
        }

        /**
         * @brief Forces an update of all limit switches.
         */
        void _checkAllLimitPins(void)
        {
            for (uint8_t index = 0; index < LIMIT_PINS_COUNT; ++index)
                _syncLimitPinState(index);
        }

        /**
         * @brief Master logic for limit switch interrupts.
         * When an interrupt fires, this function looks up which motor this switch belongs to,
         * checks if the motor is currently running towards the switch, and if so, forces a stop.
         */
        void _handleLimitISR(uint8_t limitIdx)
        {
            _syncLimitPinState(limitIdx); // Syncs the new physical state

            const LimitMapping &mapping = limitMaps[limitIdx]; // Find the associated motor
            DueStepper &motor           = steppers[mapping.motorIdx];

            // Safety check: Only stop if the pin is active, motor is running, AND going the wrong way.
            if (limitStates[limitIdx] && motor.running() && motor.direction() == mapping.stopDirection)
            {
                motor.stop(MotorStopReason::LimitStopReached);
            }
        }

        // Hardware Interrupt Routines (ISRs) for each limit pin
        void limitISRHandlerLeftCamX()
        {
            _handleLimitISR(LEFT_CAM_X_LEFT_LIMIT_IDX);
        }
        void limitISRHandlerLeftCamY()
        {
            _handleLimitISR(LEFT_CAM_Y_FRONT_LIMIT_IDX);
        }
        void limitISRHandlerRightCamX()
        {
            Com::send(serial_packet_t((uint8_t *)("\x00\x04\x00\x05\x20\x00"), 6));
            _handleLimitISR(RIGHT_CAM_X_RIGHT_LIMIT_IDX);
        }
        void limitISRHandlerRightCamY()
        {
            Com::send(serial_packet_t((uint8_t *)("\x00\x04\x00\x05\x20\x01"), 6));
            _handleLimitISR(RIGHT_CAM_Y_FRONT_LIMIT_IDX);
        }
        void limitISRHandlerXStagePos()
        {
            _handleLimitISR(X_STAGE_LEFT_LIMIT_IDX);
        }
        void limitISRHandlerYStagePos()
        {
            _handleLimitISR(Y_STAGE_FRONT_LIMIT_IDX);
        }
        void limitISRHandlerThStagePos()
        {
            _handleLimitISR(TH_STAGE_CW_LIMIT_IDX);
        }
        void limitISRHandlerXStageNeg()
        {
            _handleLimitISR(X_STAGE_RIGHT_LIMIT_IDX);
        }
        void limitISRHandlerYStageNeg()
        {
            _handleLimitISR(Y_STAGE_BACK_LIMIT_IDX);
        }
        void limitISRHandlerThStageNeg()
        {
            _handleLimitISR(TH_STAGE_CCW_LIMIT_IDX);
        }

        // -------------------------------------------------------------------------
        // DueStepper helper functions
        // -------------------------------------------------------------------------

        /**
         * @brief Configures and starts a motor to run a specific number of steps.
         */
        void _startMotor(uint8_t motorIdx, uint32_t freq, uint8_t direction, uint8_t resolution, uint32_t numberSteps)
        {
            DueStepper &motor = steppers[motorIdx];

            motor.setFrequency(freq);
            motor.setDirection(direction);
            motor.setStepFraction(resolution);
            motor.setStepTarget(numberSteps);
            motor.start();
        }

        /**
         * @brief Configures and starts a motor to run until an encoder reaches a specific target.
         */
        void _startMotorToEncoderPosition(uint8_t index, int32_t target)
        {
            DueStepper &motor       = steppers[index];
            const uint8_t direction = (target > Encoders::getValue(index)) ? HIGH : LOW;

            motor.setFrequency(FREQ_TARGET_MOTOR);
            motor.setDirection(direction);
            motor.setEncoderTarget(target);
            motor.setStepFraction(StepFraction::NONE);
            motor.start();
        }

    }

    /**
     * @brief Setup function for motors, called once during startup.
     */
    void setup()
    {
        // 1. Initialize Limit Pins as inputs
        for (uint8_t i = 0; i < LIMIT_PINS_COUNT; ++i)
            pinMode(limitPins[i], INPUT);

        // 2. Attach Hardware Interrupts
        // TRICKY PART: The Arduino Due (SAM3X8E chip) allows interrupts on ALL pins, including analog pins.
        // This means we can attach interrupts to A8, A9, and A10. The interrupt triggers when the voltage
        // crosses the digital threshold (around 1.5V).
        void (*isrArray[LIMIT_PINS_COUNT])() = {
            limitISRHandlerLeftCamX, limitISRHandlerLeftCamY, limitISRHandlerRightCamX, limitISRHandlerRightCamY,
            limitISRHandlerXStagePos, limitISRHandlerYStagePos, limitISRHandlerThStagePos,
            limitISRHandlerXStageNeg, limitISRHandlerYStageNeg, limitISRHandlerThStageNeg};

        for (uint8_t i = 0; i < LIMIT_PINS_COUNT; ++i)
            attachInterrupt(digitalPinToInterrupt(limitPins[i]), isrArray[i], CHANGE);

        // 3. Initialize Steppers
        void (*stopCbArray[MOTORS_COUNT])(MotorStopReason) = {
            &_onLeftCamXStopped, &_onLeftCamYStopped, &_onRightCamXStopped, &_onRightCamYStopped,
            &_onXStageStopped, &_onYStageStopped, &_onThStageStopped};

        for (uint8_t i = 0; i < MOTORS_COUNT; ++i)
        {
            steppers[i].setup();
            steppers[i].setTimerNVICPriority(STEPPER_NVIC_PRIORITY);
            steppers[i].attachEncoder(Encoders::getCountPtr(i));
            steppers[i].attachOnStopCallback(stopCbArray[i]);
        }

        // Sync initial limits values
        _checkAllLimitPins();
        sendAllLimitsValues();
    }

    /**
     * @brief Continuously runs in the main loop to perform periodic safety checks.
     */
    void loop()
    {
        static uint32_t prevLimitCheck = 0;
        const uint32_t now             = millis();

        // Run the safety net
        if (now - prevLimitCheck >= 100)
        {
            // Sync states to catch any missed interrupts (e.g., limits triggered manually while off)
            for (uint8_t i = 0; i < LIMIT_PINS_COUNT; ++i)
                _syncLimitPinState(i);

            // Safety net: Force stop if the interrupt somehow failed to fire but the limit is physically active
            if (alignmentEnabled)
            {
                for (uint8_t i = 0; i < LIMIT_PINS_COUNT; ++i)
                {
                    const LimitMapping &map = limitMaps[i];
                    DueStepper &motor       = steppers[map.motorIdx];

                    if (limitStates[i] && motor.running() && motor.direction() == map.stopDirection)
                    {
                        motor.stop(MotorStopReason::LimitStopReached);
                    }
                }
            }
            prevLimitCheck = now;
        }
    }

    // -------------------------------------------------------------------------
    // Commands & Controllers (Called from Serial Parsing)
    // -------------------------------------------------------------------------

    void startMotor(char *buff, int count)
    {
        if (!alignmentEnabled || count < 6)
            return;

        const uint8_t idx = KUtils::asciiByteToMotorIndex(buff[1]);

        if (idx == INVALID_MOTOR_INDEX)
            return;

        const uint8_t dir   = (buff[2] == '1') ? HIGH : LOW;
        const uint8_t res   = buff[3];
        const uint32_t freq = (uint32_t(buff[4]) << 8) | buff[5];
        uint32_t stepsCount = UINT32_MAX; // Run "indefinitely" by default

        if (freq == 0)
            return;

        if (count >= 10)
            stepsCount = (uint32_t((uint8_t)buff[6]) << 24) |
                         (uint32_t((uint8_t)buff[7]) << 16) |
                         (uint32_t((uint8_t)buff[8]) << 8) |
                         (uint8_t)buff[9];

        _startMotor(idx, freq, dir, res, stepsCount);
    }

    void moveToEncoderPosition(char *buff, int count)
    {
        if (!alignmentEnabled || count < 4)
            return;

        const uint8_t motorIndex = KUtils::asciiByteToMotorIndex(buff[1]);

        if (motorIndex == INVALID_MOTOR_INDEX || motorIndex >= MOTORS_COUNT)
            return; // Invalid index

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
        if (index == INVALID_MOTOR_INDEX || index >= MOTORS_COUNT)
            return;

        steppers[index].stop(reason);
    }

    void sendAllLimitsValues()
    {
        for (uint8_t i = 0; i < LIMIT_PINS_COUNT; ++i)
            _sendLimitStatus(i);
    }

    void enableMotor(char *buff, int count)
    {
        if (count < 3)
            return;

        const uint8_t idx = KUtils::asciiByteToMotorIndex(buff[1]);

        if (idx == INVALID_MOTOR_INDEX || idx >= MOTORS_COUNT)
            return; // Invalid index

        steppers[idx].enable((bool)buff[2]);
    }

    void lockAlignment()
    {
        alignmentEnabled = false;
        for (uint8_t i = 0; i < MOTORS_COUNT; ++i)
        {
            if (steppers[i].running())
                stopMotor(i, MotorStopReason::SoftwareOrder);
        }
    }

    void unlockAlignment()
    {
        alignmentEnabled = true;
    }
}
