#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

/* ---------------Vacuum-----------------------*/

#define SM_DISABLE   7
#define SM_DIRECTION 8
#define SM_CLOCK     9 // output PWM ??

#define SW_DISABLE   51
#define SW_DIRECTION 53
#define SW_CLOCK     13 // output PWM ??

#define SM_VACUUM 3
#define SW_VACUUM 44

#define SW_COMPRESSED_AIR        36
#define SW_SENSOR_COMPRESSED_AIR 34

/* ---------------Deck-----------------------*/

// Set

#define DECK_DISABLE   10
#define DECK_DIRECTION 11
#define DECK_CLOCK     12 // output PWM 2

// Torque return

#define DECK_COUPLE A5

// Stops Check

#define DECK_INSOLSTOP 2
#define DECK_ALIGNSTOP 14

/* ---------------Insolation---------------*/

// Set Power

#define INSOLEDS      52
#define INSOLCOURONNE 4
#define RELAYPIN      5
#define disableLDAC   46

// Temperature sensors return

#define SONDE_TEMP_INNER A0
#define SONDE_TEMP_OUTER A6

// Led + Res Voltage return

#define LED_TENS   A3
#define LED_OFFSET A2

#define LED_TENS_C   A8
#define LED_OFFSET_C A9

// Ventil Voltage return

#define FAN_TURNS A1

// Set mux

#define GET_EN 15
#define GET_A0 16
#define GET_A1 17
#define GET_A2 18

#define GET_EN_C 24
#define GET_A0_C 21
#define GET_A1_C 20
#define GET_A2_C 19

/* ----------------------- ----------------*/

//  Hard Controls

#define POW_BUTTON         50
#define ORDER_EXTINCT      48
#define ORDER_EXTINCT_10V  42
#define ORDER_EXTINCT_3_3V 40
#define ORDER_EXTINCT_5V   38
#define EMERGENCY_STOP     6

/* ----------------------- ----------------*/

/* ---------------CONVEYOR MASK---------------*/

#define ARDKO_FRONT_LEFT_LIMIT  23
#define ARDKO_FRONT_RIGHT_LIMIT 25
#define ARDKO_BACK_LEFT_LIMIT   27
#define ARDKO_BACK_RIGHT_LIMIT  29

/* ----------------------- ----------------*/

#endif