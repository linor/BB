#pragma once

#define MAIN_ENABLE 11
#define MAIN_L 10
#define MAIN_R 12

#define DOME_ENABLE 30
#define DOME_L 28
#define DOME_R 29
#define DOME_POT A16
#define DOME_SERVO_L 37
#define DOME_SERVO_R 36

#define FLYWHEEL_ENABLE 6
#define FLYWHEEL_L 5
#define FLYWHEEL_R 9

#define S2S_ENABLE 3
#define S2S_L 2
#define S2S_R 4
#define S2S_POT A17

#define DOME_SERIAL Serial8

#define RC_SERIAL Serial3
#define DEBUG_SERIAL1 Serial1
#define DEBUG_SERIAL2 Serial6

#define MPU_SERIAL Serial4
#define MPU_I2C Wire1
#define MPU_INT 38

// #define AUDIO_SERIAL Serial5
// #define AUDIO_ACT 19
// #define AUDIO_RST 18
// #define AUDIO_INPUT A9

// #define BAT_VOLTAGE A15