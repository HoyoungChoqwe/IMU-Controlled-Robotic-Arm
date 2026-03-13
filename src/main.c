#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "BOARD.h"
#include "Timers.h"
#include "Oled.h"
#include "Buttons.h"
#include "Leds.h"
#include "Servo.h"
#include "robotCommon.h"

#define STEP_DEG    5
#define DISPLAY_REFRESH_MS 50

typedef enum { SEL_GRIPPER = 0, SEL_ARM, SEL_BASE, SEL_COUNT } Selection;

static Selection selected = SEL_GRIPPER;

//show intro briefly
#define INTRO_TIME 2000

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
            LEDs_Set(0b00000011);
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
            LEDs_Set(0b00110000);
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
        // update_display();
    }
    return 0;
}
