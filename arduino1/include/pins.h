#ifndef KLOE_PINS_H
#define KLOE_PINS_H

// #1: Left camera X axis
#define LEFT_CAM_X_EN_PIN     48
#define LEFT_CAM_X_RES_M0_PIN 46
#define LEFT_CAM_X_RES_M1_PIN 44
#define LEFT_CAM_X_STEP_PIN   42
#define LEFT_CAM_X_DIR_PIN    40
#define LEFT_CAM_X_ENC_A_PIN  28
#define LEFT_CAM_X_ENC_B_PIN  26
#define LEFT_CAM_X_LIMIT_PIN  9

// #2: Left camera Y axis
#define LEFT_CAM_Y_EN_PIN     38
#define LEFT_CAM_Y_RES_M0_PIN 36
#define LEFT_CAM_Y_RES_M1_PIN 34
#define LEFT_CAM_Y_STEP_PIN   32
#define LEFT_CAM_Y_DIR_PIN    30
#define LEFT_CAM_Y_ENC_A_PIN  22
#define LEFT_CAM_Y_ENC_B_PIN  23
#define LEFT_CAM_Y_LIMIT_PIN  11

// #3: Right camera X axis
#define RIGHT_CAM_X_EN_PIN     49
#define RIGHT_CAM_X_RES_M0_PIN 47
#define RIGHT_CAM_X_RES_M1_PIN 45
#define RIGHT_CAM_X_STEP_PIN   43
#define RIGHT_CAM_X_DIR_PIN    41
#define RIGHT_CAM_X_ENC_PIN_A  29
#define RIGHT_CAM_X_ENC_PIN_B  27
#define RIGHT_CAM_X_LIMIT_PIN  12

// #4: Right camera Y axis
#define RIGHT_CAM_Y_EN_PIN     39
#define RIGHT_CAM_Y_RES_M0_PIN 37
#define RIGHT_CAM_Y_RES_M1_PIN 35
#define RIGHT_CAM_Y_STEP_PIN   33
#define RIGHT_CAM_Y_DIR_PIN    31
#define RIGHT_CAM_Y_ENC_A_PIN  25
#define RIGHT_CAM_Y_ENC_B_PIN  24
#define RIGHT_CAM_Y_LIMIT_PIN  13

// #5: Alignment stage X axis
#define STAGE_X_EN_PIN        52
#define STAGE_X_RES_M0_PIN    A0
#define STAGE_X_RES_M1_PIN    A1
#define STAGE_X_STEP_PIN      A2
#define STAGE_X_DIR_PIN       A3
#define STAGE_X_ENC_A_PIN     3
#define STAGE_X_ENC_B_PIN     2
#define STAGE_X_NEG_LIMIT_PIN A8
#define STAGE_X_POS_LIMIT_PIN 5

// #6: Alignment stage Y axis
#define STAGE_Y_EN_PIN        50
#define STAGE_Y_RES_M0_PIN    A4
#define STAGE_Y_RES_M1_PIN    A5
#define STAGE_Y_STEP_PIN      A6
#define STAGE_Y_DIR_PIN       A7
#define STAGE_Y_ENC_A_PIN     14
#define STAGE_Y_ENC_B_PIN     15
#define STAGE_Y_NEG_LIMIT_PIN A9
#define STAGE_Y_POS_LIMIT_PIN 6

// #8: Alignment stage Theta axis
#define STAGE_TH_EN_PIN        53
#define STAGE_TH_RES_M0_PIN    18
#define STAGE_TH_RES_M1_PIN    21
#define STAGE_TH_STEP_PIN      20
#define STAGE_TH_DIR_PIN       19
#define STAGE_TH_ENC_A_PIN     16
#define STAGE_TH_ENC_B_PIN     17
#define STAGE_TH_NEG_LIMIT_PIN A10
#define STAGE_TH_POS_LIMIT_PIN 8

// SPI - Focals and leds
#define LEFT_SLAVE  4
#define RIGHT_SLAVE 10

// Encoders (Bare-Metal pins reading)
// --- Left camera X
#define LEFT_CAM_X_ENC_A_VAL ((REG_PIOD_PDSR >> 3) & 0x1)
#define LEFT_CAM_X_ENC_B_VAL ((REG_PIOD_PDSR >> 1) & 0x1)
// --- Left camera Y
#define LEFT_CAM_Y_ENC_A_VAL ((REG_PIOB_PDSR >> 26) & 0x1)
#define LEFT_CAM_Y_ENC_B_VAL ((REG_PIOA_PDSR >> 14) & 0x1)
// --- Right camera X
#define RIGHT_CAM_X_ENC_A_VAL ((REG_PIOD_PDSR >> 6) & 0x1)
#define RIGHT_CAM_X_ENC_B_VAL ((REG_PIOD_PDSR >> 2) & 0x1)
// --- Right camera Y
#define RIGHT_CAM_Y_ENC_A_VAL ((REG_PIOD_PDSR >> 0) & 0x1)
#define RIGHT_CAM_Y_ENC_B_VAL ((REG_PIOA_PDSR >> 15) & 0x1)
// --- X stage
#define STAGE_X_ENC_A_VAL ((REG_PIOC_PDSR >> 28) & 0x1)
#define STAGE_X_ENC_B_VAL ((REG_PIOB_PDSR >> 25) & 0x1)
// --- Y stage
#define STAGE_Y_ENC_A_VAL ((REG_PIOD_PDSR >> 4) & 0x1)
#define STAGE_Y_ENC_B_VAL ((REG_PIOD_PDSR >> 5) & 0x1)
// --- Theta stage
#define STAGE_TH_ENC_A_VAL ((REG_PIOA_PDSR >> 13) & 0x1)
#define STAGE_TH_ENC_B_VAL ((REG_PIOA_PDSR >> 12) & 0x1)

#endif // KLOE_PINS_H
