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

#include <hardware_declarations.h>
#include <stdio.h>
#include <BOARD.h>
#include <Timers.h>
#include <Pwm.h>
#include <servo.h>
#include <delay.h>

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
    servo_set_angle(SERVO_MIN_DEG);
    delay_ms(500); //500ms delay

    printf("Starting sweep...\r\n");

    while (1) {
        // Sweep 0 -> 270 degrees
        servo_sweep(SERVO_MIN_DEG, SERVO_MAX_DEG);

        // Sweep 270 -> 0 degrees
        servo_sweep(SERVO_MAX_DEG, SERVO_MIN_DEG);
    }

    return 0;
}
