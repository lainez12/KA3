#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define VERSION "? : Arduino1 8p v2.0"

#define MAXIMUM_SERIAL_INSTRUCTION_SIZE 64
#define ANALOG_LIMIT                    310

// Stepper / Encoder config
#define MOTOR_TARGET_MOVEMENT_TOLERANCE 3
#define FREQ_TARGET_MOTOR               1000

// Priorities
#define ENCODER_NVIC_PRIORITY 0
#define STEPPER_NVIC_PRIORITY 1

#endif // DEFINITIONS_H
