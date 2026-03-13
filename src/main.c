/*
 * ECE167 Servo Control - main.c
 * Board: ST Nucleo F411RE + UCSC IO Shield
 *
 * Controls three servos via OLED + buttons:
 *   BTN1 : cycle selected servo  (GRIPPER -> ARM -> BASE -> ...)
 *   BTN2 : +5 degrees
 *   BTN3 : -5 degrees
 *   BTN4 : all servos to 90 degrees
 *
 * OLED shows current angle of Gripper, Arm, and Base,
 * with ">" marking the currently selected servo.
 */

#include <stdio.h>
#include <BOARD.h>
#include <Timers.h>
#include <Oled.h>
#include <Buttons.h>
#include "Servo.h"

#define STEP_DEG    5

typedef enum { SEL_GRIPPER = 0, SEL_ARM, SEL_BASE, SEL_COUNT } Selection;

static Selection selected = SEL_GRIPPER;

static void update_display(void)
{
    char buf[80];
    snprintf(buf, sizeof(buf),
             "%cGripper:%3d\n"
             "%cArm    :%3d\n"
             "%cBase   :%3d\n",
             (selected == SEL_GRIPPER) ? '>' : ' ', Get_Gripper(),
             (selected == SEL_ARM)     ? '>' : ' ', Get_Arm(),
             (selected == SEL_BASE)    ? '>' : ' ', Get_Base());
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

    printf("ECE167 Servo Control\r\n");

    Servo_Init();
    update_display();

    uint32_t last_display_ms = Timers_GetMilliSeconds();

    while (1) {
        uint8_t events = Buttons_CheckEvents();

        /* BTN1: cycle which servo is selected */
        if (events & BUTTON_EVENT_1DOWN) {
            selected = (Selection)((selected + 1) % SEL_COUNT);
        }

        /* BTN2: +5 degrees on selected servo */
        if (events & BUTTON_EVENT_2DOWN) {
            switch (selected) {
                case SEL_GRIPPER: Set_Gripper(Get_Gripper() + STEP_DEG); break;
                case SEL_ARM:     Set_Arm    (Get_Arm()     + STEP_DEG); break;
                case SEL_BASE:    Set_Base   (Get_Base()    + STEP_DEG); break;
                default: break;
            }
        }

        /* BTN3: -5 degrees on selected servo */
        if (events & BUTTON_EVENT_3DOWN) {
            switch (selected) {
                case SEL_GRIPPER: Set_Gripper(Get_Gripper() - STEP_DEG); break;
                case SEL_ARM:     Set_Arm    (Get_Arm()     - STEP_DEG); break;
                case SEL_BASE:    Set_Base   (Get_Base()    - STEP_DEG); break;
                default: break;
            }
        }

        /* BTN4: reset all to 90 degrees */
        if (events & BUTTON_EVENT_4DOWN) {
            Set_Arm(90);
            Set_Base(90);
            Set_Gripper(90);
        }

        /* Refresh OLED at ~10 Hz */
        uint32_t now = Timers_GetMilliSeconds();
        if ((now - last_display_ms) >= 100) {
            last_display_ms = now;
            update_display();
        }
    }
    return 0;
}
