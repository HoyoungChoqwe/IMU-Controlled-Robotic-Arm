/**
 * @file    test.c
 * @brief   Conditionally defined main functions for running hardware tests.
 *
 * Testing functionality of APIs in ../include one at a time.
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "BOARD.h"
#include "Timers.h"
#include "Oled.h"
#include "Buttons.h"
#include "Leds.h"
#include "Servo.h"
#include "FlexSensor.h"
#include "IMU.h"
#include "robotCommon.h"


/*
 * Only uncomment one at a time.
 * Defines which test runs.
 */
//#define SERVO_TEST
// #define FLEX_TEST
#define IMU_TEST

/* Common defines */
#define INTRO_TIME 2000//show intro 2sec
#define DISPLAY_REFRESH_MS 50//refresh displays every 50ms

/*
 * BEGIN CONDITIONALLY DEFINED TEST CODE MAINS
 * ----
 */

/* Controls Joints individually manually with buttons on periph. shield */
#ifdef SERVO_TEST

#define STEP_DEG    5

typedef enum { SEL_GRIPPER = 0, SEL_ARM, SEL_BASE, SEL_COUNT } Selection;

static Selection selected = SEL_GRIPPER;

//update display for test. show mode and angles
static bool updating_indicator = true;
static void update_display(void)
{
    char buf[80];
    snprintf(buf, sizeof(buf),
             "%cGripper:%3.0f\n"
             "%cArm    :%3.0f\n"
             "%cBase   :%3.0f\n"
             "updating..%c\n",
             (selected == SEL_GRIPPER) ? '>' : ' ', Get_Gripper(),
             (selected == SEL_ARM)     ? '>' : ' ', Get_Arm(),
             (selected == SEL_BASE)    ? '>' : ' ', Get_Base(),
             updating_indicator ? '.' : ' ');
    updating_indicator = !updating_indicator;
    OLED_Clear(OLED_COLOR_BLACK);
    OLED_DrawString(buf);
    OLED_Update();
}

int main(void)
{
    BOARD_Init();
    Timers_Init();
    Buttons_Init();
    OLED_Init();
    LEDs_Init();

    printf("ECE167 Servo Control\r\n");
    MAGIC_display_error_oled("ECE167 Final\nTest1\n",INTRO_TIME);

    Servo_Init();

    uint32_t last_display_ms = Timers_GetMilliSeconds();

    while (1) {
        uint8_t events = Buttons_CheckEvents();

        /* BTN1: cycle which servo is selected */
        if (events & BUTTON_EVENT_1DOWN) {
            LEDs_Set(0b11000000);
            selected = (Selection)((selected + 1) % SEL_COUNT);
        }

        /* BTN2: +5 degrees on selected servo */
        if (events & BUTTON_EVENT_2DOWN) {
            LEDs_Set(0b00110000);
            switch (selected) {
                case SEL_GRIPPER: Set_Gripper(Get_Gripper() + STEP_DEG); break;
                case SEL_ARM:     Set_Arm(Get_Arm()     + STEP_DEG); break;
                case SEL_BASE:    Set_Base(Get_Base()    + STEP_DEG); break;
                default: break;
            }
        }

        /* BTN3: -5 degrees on selected servo */
        if (events & BUTTON_EVENT_3DOWN) {
            LEDs_Set(0b00001100);
            switch (selected) {
                case SEL_GRIPPER: Set_Gripper(Get_Gripper() - STEP_DEG); break;
                case SEL_ARM:     Set_Arm(Get_Arm()     - STEP_DEG); break;
                case SEL_BASE:    Set_Base(Get_Base()    - STEP_DEG); break;
                default: break;
            }
        }

        /* BTN4: reset all to 90 degrees */
        if (events & BUTTON_EVENT_4DOWN) {
            LEDs_Set(0b00000011);
            Set_Arm(90);
            Set_Base(90);
            Set_Gripper(90);
        }

        /* Refresh OLED at 20 Hz for more responsive feedback. */
        uint32_t now = Timers_GetMilliSeconds();
        if ((now - last_display_ms) >= DISPLAY_REFRESH_MS) {
            last_display_ms = now;
            update_display();
        }
    }
    return 0;
}

#endif //SERVO_TEST

/* Displays Flex Sensor reading and finger curled state */
#ifdef FLEX_TEST

//update display for test. show mode and angles
static bool updating_indicator = true;
static void update_display(void)
{
    char buf[80];
    snprintf(buf, sizeof(buf),
             "Flex Raw:%f\n"
             "Curled  :%s\n"
             "updating..%c\n",
             FLEX_getReading(),
             (FLEX_isFingerCurled()) ? "True" : "False",
             updating_indicator ? '.' : ' ');
    updating_indicator = !updating_indicator;
    OLED_Clear(OLED_COLOR_BLACK);
    OLED_DrawString(buf);
    OLED_Update();
}


int main(void)
{
    BOARD_Init();
    Timers_Init();
    Buttons_Init();
    OLED_Init();
    LEDs_Init();
    FLEX_Init();

    printf("ECE167 FLEX test\r\n");
    MAGIC_display_error_oled("ECE167 Final\nTest2\n",INTRO_TIME);

    uint32_t last_display_ms = Timers_GetMilliSeconds();

    while (1) {
   
        /* Refresh OLED at 20 Hz for more responsive feedback. */
        uint32_t now = Timers_GetMilliSeconds();
        if ((now - last_display_ms) >= DISPLAY_REFRESH_MS) {
            last_display_ms = now;
            update_display();
        }

        if(FLEX_isFingerCurled()) {
            LEDs_Set(0b11111111);//all on
        } else {
            LEDs_Set(0b00000000);//all off
        }
    }
}
#endif //FLEX_TEST

/* Displays roll angle and range, pitch angle and range, pressing button resets baseline */
#ifdef IMU_TEST

#define RESET_MSG_DURATION 500

//update display for test. show mode and angles
static bool updating_indicator = true;
static void update_display(void)
{
    char buf[80];
    int rollDeg = (int) IMU_GetRollAngle();//cast to int to remove decimal
    int pitchDeg = (int) IMU_GetPitchAngle();
    int rollRange = IMU_GetRollRange();
    int pitchRange = IMU_GetPitchRange();

    snprintf(buf, sizeof(buf),
             "Roll :%dº\nspd:%d\n"
             "Pitch:%dº\nspd:%d\n"
             "updating..%c\n",
             rollDeg,
             rollRange,
             pitchDeg,
             pitchRange,
             updating_indicator ? '.' : ' ');
    updating_indicator = !updating_indicator;
    OLED_Clear(OLED_COLOR_BLACK);
    OLED_DrawString(buf);
    OLED_Update();
}


int main(void)
{
    BOARD_Init();
    Timers_Init();
    Buttons_Init();
    OLED_Init();
    LEDs_Init();
    IMU_Init();

    printf("ECE167 IMU test\r\n");
    MAGIC_display_error_oled("ECE167 Final\nTest3\n",INTRO_TIME);

    uint32_t last_display_ms = Timers_GetMilliSeconds();

    while (1) {

        uint8_t events = Buttons_CheckEvents();

        /* Button1 resets neutral position */
        if (events & BUTTON_EVENT_1DOWN) {
            IMU_ResetBaseline();
            MAGIC_display_error_oled("BASELINE RESET\n",RESET_MSG_DURATION);
        }
   
        /* Refresh OLED at 20 Hz for more responsive feedback. */
        uint32_t now = Timers_GetMilliSeconds();
        if ((now - last_display_ms) >= DISPLAY_REFRESH_MS) {
            last_display_ms = now;
            update_display();
        }
    }
}
#endif //IMU_TEST
