#include "vacuum.h"
#include "SerialTXHandler.h"
#include "pins.h"
#include "utils.h"

// -----------------------------------------------------------------------------
// Constants & Mappings
// -----------------------------------------------------------------------------

#define MASK_IDX  0
#define WAFER_IDX 1

namespace Vacuum
{
    namespace
    {
        // Output Mappings (Solenoids)
        const uint32_t dirPins[2]     = {SM_DIRECTION, SW_DIRECTION};
        const uint32_t disablePins[2] = {SM_DISABLE, SW_DISABLE};
        const uint32_t pwmPins[2]     = {SM_CLOCK, SW_CLOCK};

        // Input Mappings (Sensors)
        const uint32_t sensorPins[2] = {SM_VACUUM, SW_VACUUM};

        // System States
        volatile bool sensorStates[2]          = {HIGH, HIGH};
        volatile bool compressedAirSensorState = HIGH;
        bool compressedAirValveState           = false;
        uint32_t solenoidPower[2]              = {0, 0};

        /**
         * @brief Checks physical vacuum sensors and sends data if changed.
         */
        void _syncVacuumSensorState(uint8_t idx)
        {
            const bool state = digitalRead(sensorPins[idx]);
            if (state != sensorStates[idx])
            {
                sensorStates[idx]       = state;
                const uint8_t stateByte = !state + '0';
                const uint8_t buff[]    = {'V', 'C', (idx == WAFER_IDX) ? (uint8_t)'W' : (uint8_t)'M', stateByte};
                Com::send(serial_packet_t(buff, sizeof(buff)));
            }
        }

        /**
         * @brief Checks physical compressed air sensor.
         */
        void _syncCompressedAirSensorState(void)
        {
            const bool state = digitalRead(SW_SENSOR_COMPRESSED_AIR);
            if (state != compressedAirSensorState)
            {
                compressedAirSensorState = state;
                const uint8_t stateByte  = !state + '0'; // Active low mapping
                const uint8_t buff[]     = {'V', 'A', 'C', stateByte};
                Com::send(serial_packet_t(buff, sizeof(buff)));
            }
        }

        // ISR Handlers
        void isrVacuumMask()
        {
            _syncVacuumSensorState(MASK_IDX);
        }
        void isrVacuumWafer()
        {
            _syncVacuumSensorState(WAFER_IDX);
        }
        void isrAirSensor()
        {
            _syncCompressedAirSensorState();
        }
    }

    void setup(void)
    {
        // 1. Outputs
        for (uint8_t i = 0; i < 2; ++i)
        {
            pinMode(disablePins[i], OUTPUT);
            pinMode(dirPins[i], OUTPUT);
            pinMode(pwmPins[i], OUTPUT);

            digitalWrite(disablePins[i], HIGH);
            digitalWrite(dirPins[i], LOW);
        }

        pinMode(SW_COMPRESSED_AIR, OUTPUT);
        digitalWrite(SW_COMPRESSED_AIR, LOW);

        // 2. Inputs & Hardware Interrupts
        pinMode(SM_VACUUM, INPUT);
        pinMode(SW_VACUUM, INPUT);
        pinMode(SW_SENSOR_COMPRESSED_AIR, INPUT);

        sensorStates[MASK_IDX]   = digitalRead(SM_VACUUM);
        sensorStates[WAFER_IDX]  = digitalRead(SW_VACUUM);
        compressedAirSensorState = digitalRead(SW_SENSOR_COMPRESSED_AIR);

        attachInterrupt(digitalPinToInterrupt(SM_VACUUM), isrVacuumMask, CHANGE);
        attachInterrupt(digitalPinToInterrupt(SW_VACUUM), isrVacuumWafer, CHANGE);
        attachInterrupt(digitalPinToInterrupt(SW_SENSOR_COMPRESSED_AIR), isrAirSensor, CHANGE);
    }

    void loop(void)
    {
        static unsigned long prevSafetyTime = 0;

        // 100ms Software Fallback Net
        if (millis() - prevSafetyTime >= 100)
        {
            prevSafetyTime = millis();
            _syncVacuumSensorState(MASK_IDX);
            _syncVacuumSensorState(WAFER_IDX);
            _syncCompressedAirSensorState();
        }
    }

    void setSolenoid(char *buff, int count)
    {
        if (count < 5)
            return;

        // "VE" + <M or W> + <Dir (0 or 1)> + <Power 0-4095>
        const uint8_t idx = (buff[2] == 'M' || buff[2] == 'm') ? MASK_IDX : WAFER_IDX;
        const uint8_t dir = (buff[3] == '1') ? HIGH : LOW;

        uint32_t value = 0;
        for (int i = 4; i < count; ++i)
        {
            if (buff[i] >= '0' && buff[i] <= '9')
                value = (value * 10) + (buff[i] - '0');
        }

        if (value > 4095)
            value = 4095;

        // Atomicity lock for safe driver pin update
        uint32_t primask = __get_PRIMASK();
        __disable_irq();

        digitalWrite(dirPins[idx], dir);
        digitalWrite(disablePins[idx], LOW);

        if (solenoidPower[idx] != value)
        {
            // Note: AnalogWrite generates a legacy software PWM natively on Due
            analogWrite(pwmPins[idx], value);
            solenoidPower[idx] = value;
        }

        __set_PRIMASK(primask);

        getSolenoidPower(buff, count);
    }

    void toggleCompressedAirValveState(char *buff, int count)
    {
        if (count > 3)
            return;

        const bool requestState = (buff[2] == '1');

        if (compressedAirValveState != requestState)
        {
            digitalWrite(SW_COMPRESSED_AIR, requestState ? HIGH : LOW);
            compressedAirValveState = requestState;
            sendCompressedAirValveState();
        }
    }

    void sendStateSensor(char *buff, int count)
    {
        if (count != 4)
            return;

        if (buff[3] == 'M' || buff[3] == 'm')
        {
            const uint8_t stateByte = !digitalRead(SM_VACUUM) + '0';
            const uint8_t buffer[]  = {'V', 'C', (uint8_t)buff[3], stateByte};
            Com::send(serial_packet_t(buffer, sizeof(buffer)));
        }
        else if (buff[3] == 'W' || buff[3] == 'w')
        {
            const uint8_t stateByte = !digitalRead(SW_VACUUM) + '0';
            const uint8_t buffer[]  = {'V', 'C', (uint8_t)buff[3], stateByte};
            Com::send(serial_packet_t(buffer, sizeof(buffer)));
        }
    }

    void getSolenoidPower(const char *buff, uint32_t count)
    {
        if (count < 3)
            return;

        bool isMask = false;
        if (buff[2] == 'M' || buff[2] == 'm')
            isMask = true;
        else if (count >= 4 && buff[0] == '?' && (buff[3] == 'M' || buff[3] == 'm'))
            isMask = true;

        const uint8_t idx    = isMask ? MASK_IDX : WAFER_IDX;
        const uint8_t state  = digitalRead(dirPins[idx]);
        const uint32_t power = solenoidPower[idx];

        uint8_t payload[64];
        uint16_t len = 0;

        payload[len++] = 'V';
        payload[len++] = 'E';
        payload[len++] = isMask ? 'M' : 'W';
        payload[len++] = state ? '1' : '0';

        len += uintToAscii(&payload[len], power);
        Com::send(serial_packet_t(payload, len));
    }

    void sendCompressedAirValveState(void)
    {
        const uint8_t stateByte = digitalRead(SW_COMPRESSED_AIR) + '0';
        const uint8_t buff[]    = {'V', 'V', 'A', 'C', stateByte};
        Com::send(serial_packet_t(buff, sizeof(buff)));
    }

    void sendCompressedAirSensorState(void)
    {
        const uint8_t stateByte = !digitalRead(SW_SENSOR_COMPRESSED_AIR) + '0';
        const uint8_t buff[]    = {'V', 'A', 'C', stateByte};
        Com::send(serial_packet_t(buff, sizeof(buff)));
    }
}
