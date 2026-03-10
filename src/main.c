/*
 * ECE167 Servo Test
 * Board: ST Nucleo F411RE + UCSC IO Shield
 * PWM output: IO Shield Pin 58 (PWM5 = TIM4_CH3 = PB8)
 *
 * Uncomment ONE of the following to select the test to run:
 */
//#define TEST_BASE       // Sweep test: TD8120 270-deg, full 0<->270 sweep
//#define TEST_GRIPPER    // Gripper test: TD8120 270-deg, interactive debug
#define TEST_ARM        // Arm test:     TD8120 180-deg, interactive debug

#include <stdio.h>
#include <BOARD.h>
#include <Timers.h>
#include <Pwm.h>

// Shared: 50Hz period for both tests (1MHz timer clock, 1 tick = 1us)
#define SERVO_PERIOD_TICKS  19999   // ARR: 1,000,000/50 - 1 = 19999

static void delay_ms(uint32_t ms)
{
    uint32_t start = Timers_GetMilliSeconds();
    while ((Timers_GetMilliSeconds() - start) < ms);
}


/* ============================================================
 * TEST_BASE: TD8120 270-degree sweep test
 * Servo spec: 0.5ms (0deg) to 2.5ms (270deg)
 * Sweeps 0 -> 270 -> 0 continuously
 * ============================================================ */
#ifdef TEST_BASE

#define BASE_MIN_US         500     // 0.5ms = 0 degrees
#define BASE_MAX_US         2500    // 2.5ms = 270 degrees
#define BASE_MAX_DEG        270.0f
#define SWEEP_STEP_DEG      1
#define SWEEP_STEP_DELAY_MS 20      // ~5.4 sec per full sweep

static void servo_set_angle(float angle)
{
    if (angle < 0.0f)        angle = 0.0f;
    if (angle > BASE_MAX_DEG) angle = BASE_MAX_DEG;
    uint32_t ticks = (uint32_t)(BASE_MIN_US + (angle / BASE_MAX_DEG) * (BASE_MAX_US - BASE_MIN_US));
    TIM4->CCR3 = ticks;
    TIM1->CCR4 = ticks;
}

int main(void)
{
    BOARD_Init();
    Timers_Init();

    printf("TEST_BASE: TD8120 270-deg sweep\r\n");

    if (PWM_Init() == ERROR)            { printf("ERROR: PWM_Init\r\n");       while (1); }
    if (PWM_AddPin(PWM_5) == ERROR)     { printf("ERROR: PWM_AddPin 5\r\n");   while (1); }
    if (PWM_AddPin(PWM_3) == ERROR)     { printf("ERROR: PWM_AddPin 3\r\n");   while (1); }

    TIM4->ARR = SERVO_PERIOD_TICKS;
    TIM1->ARR = SERVO_PERIOD_TICKS;

    servo_set_angle(0.0f);
    delay_ms(500);
    printf("Sweeping...\r\n");

    float angle;
    while (1) {
        for (angle = 0.0f; angle <= BASE_MAX_DEG; angle += SWEEP_STEP_DEG) {
            servo_set_angle(angle);
            delay_ms(SWEEP_STEP_DELAY_MS);
        }
        for (angle = BASE_MAX_DEG; angle >= 0.0f; angle -= SWEEP_STEP_DEG) {
            servo_set_angle(angle);
            delay_ms(SWEEP_STEP_DELAY_MS);
        }
    }
    return 0;
}

#endif /* TEST_BASE */


/* ============================================================
 * TEST_GRIPPER: TD8120 270-degree gripper interactive debug
 * Servo spec: 0.5ms (0deg) to 2.5ms (270deg)
 * On power-up: center at 135 deg
 *
 * OLED: real-time angle + mode display
 * BTN1 = -2 deg & stop        BTN2 = +2 deg & stop
 * BTN3 = center (135 deg)     BTN4 = start SWEEP / stop in place
 *
 * SWEEP: current -> 180 -> 0 -> 180 -> ... (bouncing)
 * During SWEEP, BTN1/2 adjust+stop, BTN3 center+stop, BTN4 stop in place
 * ============================================================ */
#ifdef TEST_GRIPPER

#include <Oled.h>
#include <Buttons.h>

#define GRIP_MIN_US         500     // 0.5ms = 0 degrees
#define GRIP_MAX_US         2500    // 2.5ms = 270 degrees
#define GRIP_MAX_DEG        270.0f
#define GRIP_CENTER_DEG     135.0f
#define SWEEP_STEP_DEG      1.0f
#define SWEEP_STEP_DELAY_MS 20      // ~3.6 sec per full 0-180 sweep
#define SWEEP_MAX_DEG       180.0f  // sweep bounces between 0 and 180

typedef enum { MODE_MANUAL = 0, MODE_SWEEP } GripMode;

static float   current_angle = GRIP_CENTER_DEG;
static GripMode mode         = MODE_MANUAL;
static int     sweep_dir     = 1;   // +1 = towards 180, -1 = towards 0

static void servo_set_angle(float angle)
{
    if (angle < 0.0f)         angle = 0.0f;
    if (angle > GRIP_MAX_DEG) angle = GRIP_MAX_DEG;
    current_angle = angle;
    uint32_t ticks = (uint32_t)(GRIP_MIN_US + (angle / GRIP_MAX_DEG) * (GRIP_MAX_US - GRIP_MIN_US));
    TIM4->CCR3 = ticks;
    TIM1->CCR4 = ticks;
}

static void update_display(void)
{
    char buf[64];
    OLED_Clear(OLED_COLOR_BLACK);
    snprintf(buf, sizeof(buf),
             "Servo Debug\nAngle:%.1f deg\n%s\n",
             current_angle,
             mode == MODE_MANUAL ? "Mode:MANUAL" : "Mode:SWEEP ");
    OLED_DrawString(buf);
    OLED_Update();
}

int main(void)
{
    BOARD_Init();
    Timers_Init();
    Buttons_Init();
    OLED_Init();

    printf("TEST_GRIPPER: Interactive Debug\r\n");

    if (PWM_Init() == ERROR)            { printf("ERROR: PWM_Init\r\n");       while (1); }
    if (PWM_AddPin(PWM_5) == ERROR)     { printf("ERROR: PWM_AddPin 5\r\n");   while (1); }
    if (PWM_AddPin(PWM_3) == ERROR)     { printf("ERROR: PWM_AddPin 3\r\n");   while (1); }

    TIM4->ARR = SERVO_PERIOD_TICKS;
    TIM1->ARR = SERVO_PERIOD_TICKS;

    servo_set_angle(GRIP_CENTER_DEG);
    update_display();
    delay_ms(500);

    uint32_t last_step_ms    = Timers_GetMilliSeconds();
    uint32_t last_display_ms = Timers_GetMilliSeconds();

    while (1) {
        uint8_t events = Buttons_CheckEvents();

        if (mode == MODE_MANUAL) {
            if      (events & BUTTON_EVENT_1DOWN) { servo_set_angle(current_angle - 2.0f); }
            else if (events & BUTTON_EVENT_2DOWN) { servo_set_angle(current_angle + 2.0f); }
            else if (events & BUTTON_EVENT_3DOWN) { servo_set_angle(GRIP_CENTER_DEG); }
            else if (events & BUTTON_EVENT_4DOWN) {
                mode = MODE_SWEEP;
                sweep_dir    = 1;   // first head towards 180
                last_step_ms = Timers_GetMilliSeconds();
            }
        } else { /* MODE_SWEEP */
            if      (events & BUTTON_EVENT_1DOWN) { servo_set_angle(current_angle - 2.0f); mode = MODE_MANUAL; }
            else if (events & BUTTON_EVENT_2DOWN) { servo_set_angle(current_angle + 2.0f); mode = MODE_MANUAL; }
            else if (events & BUTTON_EVENT_3DOWN) { servo_set_angle(GRIP_CENTER_DEG);      mode = MODE_MANUAL; }
            else if (events & BUTTON_EVENT_4DOWN) { mode = MODE_MANUAL; /* stop in place */ }
            else {
                uint32_t now = Timers_GetMilliSeconds();
                if ((now - last_step_ms) >= SWEEP_STEP_DELAY_MS) {
                    last_step_ms = now;
                    float next = current_angle + sweep_dir * SWEEP_STEP_DEG;
                    if      (next >= SWEEP_MAX_DEG) { next = SWEEP_MAX_DEG; sweep_dir = -1; }
                    else if (next <= 0.0f)          { next = 0.0f;          sweep_dir =  1; }
                    servo_set_angle(next);
                }
            }
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

#endif /* TEST_GRIPPER */


/* ============================================================
 * TEST_ARM: TD8120 180-degree arm interactive debug
 * Servo spec: 0.5ms (0deg) to 2.5ms (180deg)
 * On power-up: center at 90 deg
 *
 * OLED: real-time angle + mode display
 * BTN1 = -2 deg & stop        BTN2 = +2 deg & stop
 * BTN3 = center (90 deg)      BTN4 = start SWEEP / stop in place
 *
 * SWEEP: current -> 180 -> 0 -> 180 -> ... (bouncing)
 * During SWEEP, BTN1/2 adjust+stop, BTN3 center+stop, BTN4 stop in place
 * ============================================================ */
#ifdef TEST_ARM

#include <Oled.h>
#include <Buttons.h>

#define ARM_MIN_US          500     // 0.5ms = 0 degrees
#define ARM_MAX_US          2500    // 2.5ms = 180 degrees
#define ARM_MAX_DEG         180.0f
#define ARM_CENTER_DEG      90.0f
#define ARM_STEP_DEG        1.0f
#define ARM_STEP_DELAY_MS   20      // ~3.6 sec per full 0-180 sweep

typedef enum { ARM_MANUAL = 0, ARM_SWEEP } ArmMode;

static float   arm_angle = ARM_CENTER_DEG;
static ArmMode arm_mode  = ARM_MANUAL;
static int     arm_dir   = 1;   // +1 = towards 180, -1 = towards 0

static void arm_set_angle(float angle)
{
    if (angle < 0.0f)        angle = 0.0f;
    if (angle > ARM_MAX_DEG) angle = ARM_MAX_DEG;
    arm_angle = angle;
    uint32_t ticks        = (uint32_t)(ARM_MIN_US + (angle / ARM_MAX_DEG) * (ARM_MAX_US - ARM_MIN_US));
    uint32_t ticks_mirror = (ARM_MIN_US + ARM_MAX_US) - ticks;  // mirror: (ARM_MAX_DEG - angle)
    TIM4->CCR3 = ticks;         // Pin 58: primary
    TIM1->CCR4 = ticks_mirror;  // Pin 56: mirrored (back-to-back mount)
}

static void arm_update_display(void)
{
    char buf[64];
    OLED_Clear(OLED_COLOR_BLACK);
    snprintf(buf, sizeof(buf),
             "Arm Debug\nAngle:%.1f deg\n%s\n",
             arm_angle,
             arm_mode == ARM_MANUAL ? "Mode:MANUAL" : "Mode:SWEEP ");
    OLED_DrawString(buf);
    OLED_Update();
}

int main(void)
{
    BOARD_Init();
    Timers_Init();
    Buttons_Init();
    OLED_Init();

    printf("TEST_ARM: Interactive Debug\r\n");

    if (PWM_Init() == ERROR)            { printf("ERROR: PWM_Init\r\n");       while (1); }
    if (PWM_AddPin(PWM_5) == ERROR)     { printf("ERROR: PWM_AddPin 5\r\n");   while (1); }
    if (PWM_AddPin(PWM_3) == ERROR)     { printf("ERROR: PWM_AddPin 3\r\n");   while (1); }

    TIM4->ARR = SERVO_PERIOD_TICKS;
    TIM1->ARR = SERVO_PERIOD_TICKS;

    arm_set_angle(ARM_CENTER_DEG);
    arm_update_display();
    delay_ms(500);

    uint32_t last_step_ms    = Timers_GetMilliSeconds();
    uint32_t last_display_ms = Timers_GetMilliSeconds();

    while (1) {
        uint8_t events = Buttons_CheckEvents();

        if (arm_mode == ARM_MANUAL) {
            if      (events & BUTTON_EVENT_1DOWN) { arm_set_angle(arm_angle - 2.0f); }
            else if (events & BUTTON_EVENT_2DOWN) { arm_set_angle(arm_angle + 2.0f); }
            else if (events & BUTTON_EVENT_3DOWN) { arm_set_angle(ARM_CENTER_DEG); }
            else if (events & BUTTON_EVENT_4DOWN) {
                arm_mode     = ARM_SWEEP;
                arm_dir      = 1;   // first head towards 180
                last_step_ms = Timers_GetMilliSeconds();
            }
        } else { /* ARM_SWEEP */
            if      (events & BUTTON_EVENT_1DOWN) { arm_set_angle(arm_angle - 2.0f); arm_mode = ARM_MANUAL; }
            else if (events & BUTTON_EVENT_2DOWN) { arm_set_angle(arm_angle + 2.0f); arm_mode = ARM_MANUAL; }
            else if (events & BUTTON_EVENT_3DOWN) { arm_set_angle(ARM_CENTER_DEG);   arm_mode = ARM_MANUAL; }
            else if (events & BUTTON_EVENT_4DOWN) { arm_mode = ARM_MANUAL; /* stop in place */ }
            else {
                uint32_t now = Timers_GetMilliSeconds();
                if ((now - last_step_ms) >= ARM_STEP_DELAY_MS) {
                    last_step_ms = now;
                    float next = arm_angle + arm_dir * ARM_STEP_DEG;
                    if      (next >= ARM_MAX_DEG) { next = ARM_MAX_DEG; arm_dir = -1; }
                    else if (next <= 0.0f)        { next = 0.0f;        arm_dir =  1; }
                    arm_set_angle(next);
                }
            }
        }

        /* Refresh OLED at ~10 Hz */
        uint32_t now = Timers_GetMilliSeconds();
        if ((now - last_display_ms) >= 100) {
            last_display_ms = now;
            arm_update_display();
        }
    }
    return 0;
}

#endif /* TEST_ARM */
