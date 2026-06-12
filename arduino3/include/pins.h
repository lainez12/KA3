#ifndef PINS_H
#define PINS_H
// Sensors
#define CALG A4
#define CALD A9
#define CALA A5
// Convoyeur masque(Tiroir Porte-Masque)
#define MCM_nEn  49
#define MCM_M0   47
#define MCM_M1   45
#define MCM_STEP 43
#define MCM_DIR  41
#define BMCM0    9
#define BMCM1    11
#define BMCM2    12
#define BMCM3    13
// Convoyeur wafer (Tiroir Porte-Wafer)
#define MCW_nEn  48
#define MCW_M0   46
#define MCW_M1   44
#define MCW_STEP 42
#define MCW_DIR  40
#define BMCW0    38
#define BMCW1    36
#define BMCW2    34
// Motor AutoLevel Gauche (MALG)
#define MALG_nEn  51
#define MALG_M0   3
#define MALG_M1   2
#define MALG_M2   39
#define MALG_STEP 14
#define MALG_DIR  15
#define BNALG     A10
#define BPALG     7
// Motor AutoLevel Droit (MALD)
#define MALD_nEn  52
#define MALD_M0   A0
#define MALD_M1   A1
#define MALD_M2   37
#define MALD_STEP A2
#define MALD_DIR  A3
#define BNALD     A8
#define BPALD     5
// Motor AutoLevel Arrière (MALA)
#define MALA_nEn  53
#define MALA_M0   18
#define MALA_M1   21
#define MALA_M2   35
#define MALA_STEP 20
#define MALA_DIR  19
#define BNALA     A11
#define BPALA     8
// Encoders
#define MALG_codA 28u
#define MALG_codB 26u
#define MALD_codA 25u
#define MALD_codB 24u
#define MALA_codA 29u
#define MALA_codB 27u
#define MCM_codA  22u
#define MCM_codB  23u
#define MCW_codA  32u
#define MCW_codB  30u
// Encoders (Bare-Metal pins reading)
#define MALG_codA_VAL ((REG_PIOD_PDSR >> 3) & 0x1)
#define MALG_codB_VAL ((REG_PIOD_PDSR >> 1) & 0x1)
#define MALD_codA_VAL ((REG_PIOD_PDSR >> 0) & 0x1)
#define MALD_codB_VAL ((REG_PIOA_PDSR >> 15) & 0x1)
#define MALA_codA_VAL ((REG_PIOD_PDSR >> 6) & 0x1)
#define MALA_codB_VAL ((REG_PIOD_PDSR >> 2) & 0x1)
#define MCM_codA_VAL  ((REG_PIOB_PDSR >> 26) & 0x1)
#define MCM_codB_VAL  ((REG_PIOA_PDSR >> 14) & 0x1)
#define MCW_codA_VAL  ((REG_PIOD_PDSR >> 10) & 0x1)
#define MCW_codB_VAL  ((REG_PIOD_PDSR >> 9) & 0x1)

// #define MCM_codA 32
// #define MCM_codB 30
// #define MCW_codA 22
// #define MCW_codB 23
// Zone Masking Security (Butées sécurité)
#define LIMITE_ZONE1 A6
#define LIMITE_ZONE2 A7
#define LIMITE_ZONE3 6

// Causes of motors stopping
#define STOPBY_ORDER_SOFT      0
#define STOPBY_NB_STEP         1
#define STOPBY_POSITION_CODER  2
#define STOPBY_BUTEE           3
#define STOPBY_TARGET_EXCEEDED 4
#endif