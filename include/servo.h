//Servo control fuctions
#pragma once

#include <hardware_declarations.h>
#include <Timers.h>
#include <delay.h>

/*
 * Set servo angle by writing TIM4->CCR3 directly (PWM5 = TIM4_CH3).
 * angle: 0.0 to 180.0 degrees
 */
static void servo_set_angle(float angle)
{
    if (angle < SERVO_MIN_DEG)          angle = SERVO_MIN_DEG;
    if (angle > SERVO_MAX_DEG) angle = SERVO_MAX_DEG;
    uint32_t ticks = (uint32_t)(SERVO_MIN_US + (angle / SERVO_MAX_DEG) * (SERVO_MAX_US - SERVO_MIN_US));
    TIM4->CCR3 = ticks;
}

static void servo_sweep(float start_angle, float end_angle)
{
    float angle;

    if (start_angle <= end_angle) {
        for (angle = start_angle; angle <= end_angle; angle += SWEEP_STEP_DEG) 
        {
            servo_set_angle(angle);
            delay_ms(SWEEP_STEP_DELAY_MS);
        }
    } else {
        for (angle = start_angle; angle >= end_angle; angle -= SWEEP_STEP_DEG) 
        {
            servo_set_angle(angle);
            delay_ms(SWEEP_STEP_DELAY_MS);
        }
    }
}
