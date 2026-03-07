/*
 * ECE167 Servo Test - TD8120 Servo Sweep
 * Board: ST Nucleo F411RE + UCSC IO Shield
 *
 * Servo PWM spec: 50Hz, 0.5ms (0deg) to 2.5ms (270deg) pulse width (270-degree version)
 * Connect servo signal wire to IO Shield Pin 58 (PWM5 = TIM4_CH3 = PB8)
 *
 * Since the PWM library's PWM_SetFrequency() minimum is 100Hz, we manually
 * set TIM4->ARR after init to achieve 50Hz (timer runs at 1MHz, ARR=19999).
 * CCR is set directly in microsecond ticks (1 tick = 1us at 1MHz).
 */

#include <stdio.h>
#include <BOARD.h>
#include <Timers.h>
#include <Pwm.h>

// Servo timing constants (1MHz timer clock -> 1 tick = 1us)
#define SERVO_PERIOD_TICKS  19999   // ARR for 50Hz: 1,000,000/50 - 1 = 19999
#define SERVO_MIN_US        500     // 0.5ms pulse = 0 degrees   (270-deg version)
#define SERVO_MAX_US        2500    // 2.5ms pulse = 270 degrees (270-deg version)
#define SERVO_MAX_DEG       270.0f

// Step size and delay per step for the sweep
#define SWEEP_STEP_DEG      1       // 1 degree per step
#define SWEEP_STEP_DELAY_MS 20      // 20ms per step (~5.4 sec for full 0-270 sweep)

/*
 * Set servo angle by writing TIM4->CCR3 directly (PWM5 = TIM4_CH3).
 * angle: 0.0 to 180.0 degrees
 */
static void servo_set_angle(float angle)
{
    if (angle < 0.0f)          angle = 0.0f;
    if (angle > SERVO_MAX_DEG) angle = SERVO_MAX_DEG;
    uint32_t ticks = (uint32_t)(SERVO_MIN_US + (angle / SERVO_MAX_DEG) * (SERVO_MAX_US - SERVO_MIN_US));
    TIM4->CCR3 = ticks;
}

/*
 * Blocking millisecond delay using Timers_GetMilliSeconds().
 * Handles uint32_t wraparound correctly via unsigned subtraction.
 */
static void delay_ms(uint32_t ms)
{
    uint32_t start = Timers_GetMilliSeconds();
    while ((Timers_GetMilliSeconds() - start) < ms);
}

int main(void)
{
    BOARD_Init();
    Timers_Init();

    printf("TD8120 Servo Sweep Test\r\n");
    printf("Connect servo signal to IO Shield Pin 58 (PWM5)\r\n");

    // Init PWM library and enable PWM_5 (TIM4_CH3, GPIO PB8)
    if (PWM_Init() == ERROR) {
        printf("ERROR: PWM_Init failed\r\n");
        while (1);
    }
    if (PWM_AddPin(PWM_5) == ERROR) {
        printf("ERROR: PWM_AddPin failed\r\n");
        while (1);
    }

    // Override period to 50Hz (library minimum is 100Hz, so set ARR directly)
    // At 1MHz timer clock: period = (ARR+1) ticks -> 20000 ticks = 20ms = 50Hz
    TIM4->ARR = SERVO_PERIOD_TICKS;

    // Start at 0 degrees
    servo_set_angle(0.0f);
    delay_ms(500);

    printf("Starting sweep...\r\n");

    float angle;
    while (1) {
        // Sweep 0 -> 270 degrees
        for (angle = 0.0f; angle <= SERVO_MAX_DEG; angle += SWEEP_STEP_DEG) {
            servo_set_angle(angle);
            delay_ms(SWEEP_STEP_DELAY_MS);
        }

        // Sweep 270 -> 0 degrees
        for (angle = SERVO_MAX_DEG; angle >= 0.0f; angle -= SWEEP_STEP_DEG) {
            servo_set_angle(angle);
            delay_ms(SWEEP_STEP_DELAY_MS);
        }
    }

    return 0;
}