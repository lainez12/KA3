#ifndef PINS_H
#define PINS_H

// Sensors
#define FORCE_SENS_LEFT_PIN  A4
#define FORCE_RIGHT_SENS_PIN A9
#define FORCE_BACK_SENS_PIN  A5

// Mask Conveyor
#define MASK_CONV_EN_PIN        49
#define MASK_CONV_RES_M0_PIN    47
#define MASK_CONV_RES_M1_PIN    45
#define MASK_CONV_STEP_PIN      43
#define MASK_CONV_DIR_PIN       41
#define MASK_CONV_LIMIT_CM0_PIN 9
#define MASK_CONV_LIMIT_CM1_PIN 11
#define MASK_CONV_LIMIT_CM2_PIN 12
#define MASK_CONV_LIMIT_CM3_PIN 13
#define MASK_ENC_A_PIN          22u
#define MASK_ENC_B_PIN          23u

// Wafer Conveyor
#define WAFER_CONV_EN_PIN        48
#define WAFER_CONV_RES_M0_PIN    46
#define WAFER_CONV_RES_M1_PIN    44
#define WAFER_CONV_STEP_PIN      42
#define WAFER_CONV_DIR_PIN       40
#define WAFER_CONV_LIMIT_CW0_PIN 38
#define WAFER_CONV_LIMIT_CW1_PIN 36
#define WAFER_CONV_LIMIT_CW2_PIN 34
#define WAFER_ENC_A_PIN          32u
#define WAFER_ENC_B_PIN          30u

// Z Left Motor
#define Z_LEFT_EN_PIN       51
#define Z_LEFT_RES_M0_PIN   3
#define Z_LEFT_RES_M1_PIN   2
#define Z_LEFT_RES_M2_PIN   39
#define Z_LEFT_STEP_PIN     14
#define Z_LEFT_DIR_PIN      15
#define Z_LEFT_LO_LIMIT_PIN A10
#define Z_LEFT_HI_LIMIT_PIN 7
#define Z_LEFT_ENC_A_PIN    28u
#define Z_LEFT_ENC_B_PIN    26u

// Z Right Motor
#define Z_RIGHT_EN_PIN       52
#define Z_RIGHT_RES_M0_PIN   A0
#define Z_RIGHT_RES_M1_PIN   A1
#define Z_RIGHT_RES_M2_PIN   37
#define Z_RIGHT_STEP_PIN     A2
#define Z_RIGHT_DIR_PIN      A3
#define Z_RIGHT_LO_LIMIT_PIN A8
#define Z_RIGHT_HI_LIMIT_PIN 5
#define Z_RIGHT_ENC_A_PIN    25u
#define Z_RIGHT_ENC_B_PIN    24u

// Z Back Motor
#define Z_BACK_EN_PIN       53
#define Z_BACK_RES_M0_PIN   18
#define Z_BACK_RES_M1_PIN   21
#define Z_BACK_RES_M2_PIN   35
#define Z_BACK_STEP_PIN     20
#define Z_BACK_DIR_PIN      19
#define Z_BACK_LO_LIMIT_PIN A11
#define Z_BACK_HI_LIMIT_PIN 8
#define Z_BACK_ENC_A_PIN    29u
#define Z_BACK_ENC_B_PIN    27u

// Z Plan Limits
#define Z1_LIMIT       A6
#define Z2_LIMIT       A7
#define WAFER_ON_LIMIT 6

// Encoders

// Encoders (Bare-Metal pins reading)
// Bypasses the standard Arduino digitalRead() which takes ~4us due to pin mapping lookups.
// Directly reading the Parallel Input/Output (PIO) Controller Data Status Register (PDSR)
// takes exactly 1 CPU cycle (~11.9ns at 84MHz). Essential for high-frequency quadrature decoding.
#define MASK_ENC_A_VAL    ((REG_PIOB_PDSR >> 26) & 0x1)
#define MASK_ENC_B_VAL    ((REG_PIOA_PDSR >> 14) & 0x1)
#define WAFER_ENC_A_VAL   ((REG_PIOD_PDSR >> 10) & 0x1)
#define WAFER_ENC_B_VAL   ((REG_PIOD_PDSR >> 9) & 0x1)
#define Z_LEFT_ENC_A_VAL  ((REG_PIOD_PDSR >> 3) & 0x1)
#define Z_LEFT_ENC_B_VAL  ((REG_PIOD_PDSR >> 1) & 0x1)
#define Z_RIGHT_ENC_A_VAL ((REG_PIOD_PDSR >> 0) & 0x1)
#define Z_RIGHT_ENC_B_VAL ((REG_PIOA_PDSR >> 15) & 0x1)
#define Z_BACK_ENC_A_VAL  ((REG_PIOD_PDSR >> 6) & 0x1)
#define Z_BACK_ENC_B_VAL  ((REG_PIOD_PDSR >> 2) & 0x1)

#endif