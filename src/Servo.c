/**
 * @file    Servo.c
 * @brief   Unified servo control for ECE167 robot arm.
 *
 * Hardware map (UCSC Nucleo I/O Shield):
 *   Gripper : Pin 55  PWM2  PA10  TIM1_CH3   TD8120  20-140 deg (hard limits, physical range 0-270)
 *   Base    : Pin 57  PWM4  PB6   TIM4_CH1   TD8120  0-180 deg  (180-deg version)
 *   Arm pri : Pin 58  PWM5  PB8   TIM4_CH3   TD8120  0-180 deg
 *   Arm mir : Pin 56  PWM3  PA11  TIM1_CH4              (mirrored)
 *
 * All four channels share the same 50 Hz period (TIM1 and TIM4 ARR = 19999).
 * Pulse width range: 500 us (0 deg) to 2500 us (max deg).
 */

#include <stdio.h>
#include <BOARD.h>
#include <Pwm.h>
#include "Servo.h"

/* 50 Hz: 1 MHz timer clock, ARR = 1,000,000/50 - 1 = 19999 */
#define SERVO_PERIOD_TICKS  19999U
#define SERVO_MIN_US        500U
#define SERVO_MAX_US        2500U

#define ARM_MAX_DEG             180
#define BASE_MAX_DEG            180
/* Gripper: TD8120 270-deg physical servo, software-limited to [20, 140] */
#define GRIP_PHYSICAL_MAX_DEG   270     /* full servo range, used for pulse mapping */
#define GRIP_MIN_DEG            35      /* software lower limit */
#define GRIP_MAX_DEG            205     /* software upper limit */

static int arm_deg     = 90;
static int base_deg    = 90;
static int gripper_deg = 90;

/* Convert angle to pulse-width ticks (us == ticks at 1 MHz). */
static uint32_t angle_to_ticks(int deg, int max_deg)
{
    if (deg < 0)       deg = 0;
    if (deg > max_deg) deg = max_deg;
    return SERVO_MIN_US + (uint32_t)((deg * (SERVO_MAX_US - SERVO_MIN_US)) / max_deg);
}

void Servo_Init(void)
{
    if (PWM_Init() == ERROR)        { printf("ERROR: PWM_Init\r\n");       while (1); }
    /* Gripper: PWM_2 = TIM1_CH3 = Pin 55 */
    if (PWM_AddPin(PWM_2) == ERROR) { printf("ERROR: PWM_AddPin 2\r\n");   while (1); }
    /* Arm mirror: PWM_3 = TIM1_CH4 = Pin 56 */
    if (PWM_AddPin(PWM_3) == ERROR) { printf("ERROR: PWM_AddPin 3\r\n");   while (1); }
    /* Base: PWM_4 = TIM4_CH1 = Pin 57 */
    if (PWM_AddPin(PWM_4) == ERROR) { printf("ERROR: PWM_AddPin 4\r\n");   while (1); }
    /* Arm primary: PWM_5 = TIM4_CH3 = Pin 58 */
    if (PWM_AddPin(PWM_5) == ERROR) { printf("ERROR: PWM_AddPin 5\r\n");   while (1); }

    /* Override to 50 Hz after PWM_AddPin (which defaults to 1 kHz) */
    TIM1->ARR = SERVO_PERIOD_TICKS;
    TIM4->ARR = SERVO_PERIOD_TICKS;

    /* Move all to 90 degrees */
    Set_Arm(90);
    Set_Base(90);
    Set_Gripper(90);
}

void Set_Arm(int deg)
{
    if (deg < 0)            deg = 0;
    if (deg > ARM_MAX_DEG)  deg = ARM_MAX_DEG;
    arm_deg = deg;

    uint32_t ticks        = angle_to_ticks(deg, ARM_MAX_DEG);
    uint32_t ticks_mirror = (SERVO_MIN_US + SERVO_MAX_US) - ticks;

    TIM4->CCR3 = ticks;         /* PWM_5: Pin 58 primary  */
    TIM1->CCR4 = ticks_mirror;  /* PWM_3: Pin 56 mirrored */
}

void Set_Base(int deg)
{
    if (deg < 0)            deg = 0;
    if (deg > BASE_MAX_DEG) deg = BASE_MAX_DEG;
    base_deg = deg;

    TIM4->CCR1 = angle_to_ticks(deg, BASE_MAX_DEG); /* PWM_4: Pin 57 */
}

void Set_Gripper(int deg)
{
    if (deg < GRIP_MIN_DEG || deg > GRIP_MAX_DEG) {
        printf("ERROR: Gripper angle %d out of range [%d, %d]\r\n",
               deg, GRIP_MIN_DEG, GRIP_MAX_DEG);
        return;
    }
    gripper_deg = deg;
    /* Map against full 270-deg physical range to preserve correct pulse width */
    TIM1->CCR3 = angle_to_ticks(deg, GRIP_PHYSICAL_MAX_DEG); /* PWM_2: Pin 55 */
}

int Get_Arm(void)      { return arm_deg;     }
int Get_Base(void)     { return base_deg;    }
int Get_Gripper(void)  { return gripper_deg; }
