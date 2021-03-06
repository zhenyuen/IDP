// Pin configurations
#ifndef Config_h
#define Config_h

#include <Arduino.h>

#define DEBUG_MODE false // True to enable verbose for debugging

#define DC_M_L_PIN 3
#define DC_M_R_PIN 4
#define LS_B_PIN 5
#define LS_FM_PIN 4
#define LS_FL_PIN 3
#define LS_FR_PIN 2
#define IR_PIN 11
#define PT_NARROW_PIN A0
#define PT_WIDE_PIN A1
#define US_TRIG_PIN 13
#define US_ECHO_PIN 12
#define CLAW_PIN 9
#define CLAW_RAISER_PIN 10
#define SWITCH_PIN 6
#define CLAW_OPEN 130
#define CLAW_CLOSE 45
#define CLAW_DELAY 100
#define CLAW_RAISER_UP 145
#define CLAW_RAISER_DOWN 85
#define CLAW_RAISER_SEARCH 95
#define CLAW_RAISER_DELAY 15
#define R_LED_PIN 0
#define G_LED_PIN 1
#define A_LED_PIN 8
#define ACC_PIN A2


#define BRAKE_DELAY 200
#define LS_HIST_LEN 3
#define STEER_FACTOR 0.8
#define DEF_SPEED 240
#define DUMMY_MODULATED 1
#define DUMMY_UNMODULATED 2
#define DUMMY_ALTERNATING 3
#define IR_ANALOG_THRESHOLD 30
#define IR_CYCLES 200
#define IR_HIST_LEN 5
#define PI_SLIT_COUNT 10
#define NORMAL_STATE 1
#define ASCEND_STATE 2
#define DESCEND_STATE 3
#define ELEVATED_STATE 4
#define ASCEND_THRESHOLD -3.8
#define DESCEND_THRESHOLD 3.8
#define ACC_HIST_LEN 9
#define ROTATE_90 1500
#define ROTATE_180 3000
#define N_DUMMIES 3
#define FAILURE_LIMIT 2
#define OFF_LINE_THRESHOLD 3000
#define TIME_FROM_SLOPE 500
#define TIME_MOVE_INTO_BOX 2000
#define TIME_REALIGN_FORWARD 1000
#define TIME_REALIGN_BACKWARD 2000
#define TIME_REALIGN_TO_DUMMY_ACW 1000
#define TIME_REALIGN_TO_DUMMY_DELAY 75
#define TIME_REALIGN_TO_DUMMY_INTERVAL 100
#define TIME_LS_TRIGGER 75

// Tasks ID
#define EXIT_BOX 0
#define HEAD_TO_COLLECTION_AREA 1
#define DETECT_DUMMY 2
#define APPROACH_DUMMY 3
#define IDENTIFY_DUMMY 4
#define COLLECT_DUMMY 5
#define HEAD_TO_DROPOFF_POINT 6
#define DROP_DUMMY 7
#define EXIT_WBOX 8
#define EXIT_RBBOX 9
#define RETURN_FROM_LOC_0 10
#define RETURN_FROM_LOC_1 11
#define COMPLETED 1000



#endif
