#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "BOARD.h"
#include "Oled.h"
#include "Timers.h"
#include "robotCommon.h"

//hold time in ms
#define ERROR_SHOW_TIME 300

void MAGIC_display_error_oled(const char *msg)
{
    OLED_Clear(OLED_COLOR_BLACK);
    OLED_DrawString(msg);
    OLED_Update();
    uint32_t start = Timers_GetMilliSeconds();
    while ((Timers_GetMilliSeconds() - start) < ERROR_SHOW_TIME);
}
