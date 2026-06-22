#include "deck.h"
#include "DueDCMotor.hpp"
#include "SerialTXHandler.h"
#include "pins.h"

// -----------------------------------------------------------------------------
// Constants & Mappings
// -----------------------------------------------------------------------------

#define LIMIT_FORWARD_IDX  0
#define LIMIT_BACKWARD_IDX 1

#define MOTOR_DIR_FORWARD  LOW
#define MOTOR_DIR_BACKWARD HIGH

#define DECK_PWM_FREQUENCY      20000
#define DECK_MAX_RESOLUTION     4095
#define TORQUE_POLL_INTERVAL_MS 200
#define SAFETY_POLL_INTERVAL_MS 100

namespace Deck
{
    // The unnamed namespace acts as a strict translation unit boundary, hiding these
    // state variables and helpers from the rest of the firmware architecture.
    namespace
    {
        DueDCMotor deckMotor(
            due_dc_motor_pins_t{
                .disable   = DECK_DISABLE,
                .direction = DECK_DIRECTION,
                .pwm       = DECK_CLOCK,
                .torque    = DECK_COUPLE});

        // Hardware mapping arrays for O(1) logic resolution
        const uint8_t limitPins[2]       = {DECK_ALIGNSTOP, DECK_INSOLSTOP};
        const uint8_t limitDirections[2] = {MOTOR_DIR_FORWARD, MOTOR_DIR_BACKWARD};
        const uint8_t limitByteCodes[2]  = {'F', 'B'};

        // Internal State Caching
        volatile bool limitStates[2]      = {false, false};
        volatile uint8_t currentDirection = MOTOR_DIR_FORWARD;
        int32_t torqueLimits[2]           = {2750, 2481}; // Forward/Backward default limits

        /**
         * @brief Immediately halts the motor and notifies the host of a torque overload.
         */
        void _triggerTorqueStop()
        {
            deckMotor.stop();
            const uint8_t buffer[] = {'C', 'L', '1'};
            Com::send(serial_packet_t(buffer, sizeof(buffer)));
        }

        /**
         * @brief Reads the physical state of a limit pin.
         * The switches are active LOW mechanically, hence the inversion.
         */
        bool _getLimitPinValue(uint8_t limitIdx)
        {
            return !digitalRead(limitPins[limitIdx]);
        }

        /**
         * @brief Builds and sends the serial message reporting the state of a limit switch.
         */
        void _sendLimitStatus(uint8_t limitIdx)
        {
            const uint8_t code           = limitByteCodes[limitIdx];
            const uint8_t limitValueByte = (uint8_t)(limitStates[limitIdx] + '0');
            const uint8_t buffer[]       = {'C', '1', code, limitValueByte};

            Com::send(serial_packet_t(buffer, sizeof(buffer)));
        }

        /**
         * @brief Checks if a limit switch has changed state since we last checked.
         * Updates the cached state and notifies the host software.
         */
        void _syncLimitPinState(uint8_t limitIdx)
        {
            const bool state = _getLimitPinValue(limitIdx);

            if (state != limitStates[limitIdx])
            {
                limitStates[limitIdx] = state; // Update stored value
                _sendLimitStatus(limitIdx);
            }
        }

        /**
         * @brief Master logic for limit switch interrupts.
         * Handles instantaneous collision prevention in O(1) execution time.
         */
        void _handleLimitISR(uint8_t limitIdx)
        {
            _syncLimitPinState(limitIdx); // Syncs new physical state

            // Safety check: Only stop if the pin is active, motor is running, AND going the wrong way.
            if (limitStates[limitIdx] && deckMotor.isEnabled() && currentDirection == limitDirections[limitIdx])
            {
                deckMotor.stop();
            }
        }

        // Hardware Interrupt Routines (ISRs) for each deck limit
        void isrLimitHandlerForward()
        {
            _handleLimitISR(LIMIT_FORWARD_IDX);
        }
        void isrLimitHandlerBackward()
        {
            _handleLimitISR(LIMIT_BACKWARD_IDX);
        }
    }

    void setup(void)
    {
        deckMotor.setup(DECK_PWM_FREQUENCY, DECK_MAX_RESOLUTION);

        // Active HIGH logic defaults
        digitalWrite(DECK_DISABLE, HIGH);
        digitalWrite(DECK_DIRECTION, HIGH);

        // Initialize Limit Pins and attach hardware EXTI handlers
        for (uint8_t i = 0; i < 2; ++i)
        {
            pinMode(limitPins[i], INPUT);
            limitStates[i] = _getLimitPinValue(i); // Force sync initial state

            attachInterrupt(digitalPinToInterrupt(limitPins[i]),
                            (i == LIMIT_FORWARD_IDX) ? isrLimitHandlerForward : isrLimitHandlerBackward,
                            CHANGE);
        }
    }

    void loop(void)
    {
        const uint32_t now              = millis();
        static uint32_t prevTorqueCheck = 0;
        static uint32_t prevLimitCheck  = 0;

        // 1. Torque Monitoring Loop
        // Because analogRead() takes ~40us, doing this every 200ms is perfectly non-blocking.
        if (deckMotor.isEnabled() && (now - prevTorqueCheck >= TORQUE_POLL_INTERVAL_MS))
        {
            prevTorqueCheck      = now;
            const uint8_t dirIdx = (currentDirection == MOTOR_DIR_BACKWARD) ? LIMIT_BACKWARD_IDX : LIMIT_FORWARD_IDX;

            if (deckMotor.readTorque() > (uint32_t)torqueLimits[dirIdx])
            {
                _triggerTorqueStop();
            }
        }

        // 2. Limit Switch Software Fallback Loop
        // Syncs states periodically in case EMI transient caused a missed EXTI edge
        if (now - prevLimitCheck >= SAFETY_POLL_INTERVAL_MS)
        {
            prevLimitCheck = now;
            for (uint8_t i = 0; i < 2; ++i)
            {
                _syncLimitPinState(i);

                // Safety net force stop
                if (limitStates[i] && deckMotor.isEnabled() && currentDirection == limitDirections[i])
                {
                    deckMotor.stop();
                }
            }
        }
    }

    void startMotor(char *buff, int count)
    {
        if (count < 3)
            return;

        // Command Format: CF<Dir><Speed> (e.g. CFF2000, CFB1000) or CFS for stop.
        if (buff[2] == 'S' || buff[2] == 's')
        {
            stopMotor();
            return;
        }

        const bool dirIsFront = (buff[2] == 'F' || buff[2] == 'f');
        const bool dirIsValid = dirIsFront || (buff[2] == 'B' || buff[2] == 'b');

        if (dirIsValid)
        {
            const uint8_t targetDir = dirIsFront ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD;
            // Evaluate if movement is pre-emptively blocked by a limit switch
            const uint8_t limitIdx = dirIsFront ? LIMIT_FORWARD_IDX : LIMIT_BACKWARD_IDX;

            if (limitStates[limitIdx])
                return; // Limit is active, abort start

            int32_t speedValue = 0;
            for (int i = 3; i < count; ++i)
            {
                if (buff[i] >= '0' && buff[i] <= '9')
                    speedValue = (speedValue * 10) + (buff[i] - '0');
            }

            if (speedValue > DECK_MAX_RESOLUTION)
                speedValue = DECK_MAX_RESOLUTION;
            if (speedValue < 0)
                speedValue = 0;

            // Disable interrupts to ensure motor start transition is atomic
            uint32_t primask = __get_PRIMASK();
            __disable_irq();

            currentDirection = targetDir;
            deckMotor.setDirection(currentDirection);
            deckMotor.setSpeed(speedValue);
            deckMotor.enable(true);

            __set_PRIMASK(primask);
        }
    }

    void setTorqueLimit(char *buff, int count)
    {
        if (count < 3)
            return;

        const bool dirIsFront = (buff[1] == 'F' || buff[1] == 'f');
        const bool dirIsValid = dirIsFront || (buff[1] == 'B' || buff[1] == 'b');

        if (!dirIsValid)
            return;

        // Command format: TF<Value> or TB<Value>
        const uint8_t dirIdx = dirIsFront ? LIMIT_FORWARD_IDX : LIMIT_BACKWARD_IDX;

        int32_t value = 0;
        for (int i = 2; i < count; ++i)
        {
            if (buff[i] >= '0' && buff[i] <= '9')
                value = (value * 10) + (buff[i] - '0');
        }

        if (value > DECK_MAX_RESOLUTION)
            value = DECK_MAX_RESOLUTION;
        if (value < 0)
            value = 0;

        torqueLimits[dirIdx] = value;
    }

    void stopMotor(void)
    {
        deckMotor.stop();
    }

    void sendAllLimitsValues(void)
    {
        for (uint8_t i = 0; i < 2; ++i)
            _sendLimitStatus(i);
    }
}
