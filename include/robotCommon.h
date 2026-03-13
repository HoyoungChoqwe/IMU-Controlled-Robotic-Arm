//common functions

void MAGIC_display_error_oled(const char *msg);
//show intro briefly
// #define INTRO_TIME 2000
// static void show_intro_oled(void)
// {
//     char buf[80];
//     snprintf(buf, sizeof(buf),
//              "ECE167 Final\nTest 1\n");
//     OLED_Clear(OLED_COLOR_BLACK);
//     OLED_DrawString(buf);
//     OLED_Update();
//     uint32_t start = Timers_GetMilliSeconds();
//     while ((Timers_GetMilliSeconds() - start) < INTRO_TIME);
// }

