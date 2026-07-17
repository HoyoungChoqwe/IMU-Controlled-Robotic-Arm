#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <BOARD.h>
#include <Timers.h>

#include "Servo.h"

#define LINE_LEN 48

static void print_status(void)
{
    printf("G:%3d A:%3d B:%3d T:%3d\r\n",
           Get_Gripper(), Get_Arm(), Get_Base(), Get_Top());
}

static void print_help(void)
{
    printf("Commands:\r\n");
    printf("  a +5     add 5 degrees to paired arm servos\r\n");
    printf("  a -5     subtract 5 degrees from paired arm servos\r\n");
    printf("  a 90     set paired arm servos to 90\r\n");
    printf("  b 90     set base servo to 90\r\n");
    printf("  t 90     set top servo to 90\r\n");
    printf("  g 90     set gripper servo to 90\r\n");
    printf("  s        status\r\n");
    printf("  r        reset servos to 90\r\n");
    printf("  h        help\r\n");
}

static void read_line(char *line, size_t len)
{
    size_t index = 0;

    while (index + 1 < len) {
        int ch = getchar();
        if (ch == '\r' || ch == '\n') {
            break;
        }
        line[index++] = (char)ch;
    }
    line[index] = '\0';
}

static int current_value(const char *name)
{
    if (strcmp(name, "g") == 0) {
        return Get_Gripper();
    }
    if (strcmp(name, "a") == 0) {
        return Get_Arm();
    }
    if (strcmp(name, "b") == 0) {
        return Get_Base();
    }
    if (strcmp(name, "t") == 0) {
        return Get_Top();
    }
    return 0;
}

static int set_value(const char *name, int value)
{
    if (strcmp(name, "g") == 0) {
        Set_Gripper(value);
    } else if (strcmp(name, "a") == 0) {
        Set_Arm(value);
    } else if (strcmp(name, "b") == 0) {
        Set_Base(value);
    } else if (strcmp(name, "t") == 0) {
        Set_Top(value);
    } else {
        return 0;
    }

    return 1;
}

static void reset_servos(void)
{
    Set_Arm(90);
    Set_Base(90);
    Set_Top(90);
    Set_Gripper(90);
}

static void handle_command(char *line)
{
    char name[8] = {0};
    char value_text[16] = {0};

    for (char *p = line; *p; p++) {
        *p = (char)tolower((unsigned char)*p);
    }

    if (sscanf(line, "%7s %15s", name, value_text) < 1) {
        return;
    }

    if (strcmp(name, "s") == 0 || strcmp(name, "status") == 0) {
        print_status();
        return;
    }

    if (strcmp(name, "r") == 0 || strcmp(name, "reset") == 0) {
        reset_servos();
        print_status();
        return;
    }

    if (strcmp(name, "h") == 0 || strcmp(name, "help") == 0) {
        print_help();
        return;
    }

    if (value_text[0] == '\0') {
        printf("Missing value. Type h for help.\r\n");
        return;
    }

    int value = atoi(value_text);
    if (value_text[0] == '+' || value_text[0] == '-') {
        value += current_value(name);
    }

    if (!set_value(name, value)) {
        printf("Unknown servo '%s'. Type h for help.\r\n", name);
        return;
    }

    print_status();
}

int main(void)
{
    char line[LINE_LEN];

    BOARD_Init();
    Timers_Init();
    Servo_Init();

    printf("Manual Servo Control\r\n");
    print_help();
    print_status();

    while (1) {
        printf("> ");
        read_line(line, sizeof(line));
        handle_command(line);
    }

    return 0;
}
