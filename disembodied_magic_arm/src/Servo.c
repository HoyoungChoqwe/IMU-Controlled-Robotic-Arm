#include <stdio.h>

#include <BOARD.h>
#include <Pwm.h>

#include "Servo.h"

#define SERVO_PERIOD_TICKS 19999U
#define SERVO_MIN_US 500U
#define SERVO_MAX_US 2500U

#define ARM_MAX_DEG 180
#define TOP_MAX_DEG 180
#define BASE_MAX_DEG 180
#define GRIP_PHYSICAL_MAX_DEG 270
#define GRIP_MIN_DEG 35
#define GRIP_MAX_DEG 205

static int arm_deg = 90;
static int top_deg = 90;
static int base_deg = 90;
static int gripper_deg = 90;

static uint32_t angle_to_ticks(int deg, int max_deg)
{
    if (deg < 0)
    {
        deg = 0;
    }
    if (deg > max_deg)
    {
        deg = max_deg;
    }

    return SERVO_MIN_US + (uint32_t) ((deg * (SERVO_MAX_US - SERVO_MIN_US)) / max_deg);
}

void Servo_Init(void)
{
    if (PWM_Init() == ERROR)
    {
        printf("ERROR: PWM_Init\r\n");
        while (TRUE)
        {
        }
    }
    if (PWM_AddPin(PWM_2) == ERROR)
    {
        printf("ERROR: PWM_AddPin 2\r\n");
        while (TRUE)
        {
        }
    }
    if (PWM_AddPin(PWM_3) == ERROR)
    {
        printf("ERROR: PWM_AddPin 3\r\n");
        while (TRUE)
        {
        }
    }
    if (PWM_AddPin(PWM_4) == ERROR)
    {
        printf("ERROR: PWM_AddPin 4\r\n");
        while (TRUE)
        {
        }
    }
    if (PWM_AddPin(PWM_5) == ERROR)
    {
        printf("ERROR: PWM_AddPin 5\r\n");
        while (TRUE)
        {
        }
    }
    if (PWM_AddPin(PWM_6) == ERROR)
    {
        printf("ERROR: PWM_AddPin 6\r\n");
        while (TRUE)
        {
        }
    }

    TIM1->ARR = SERVO_PERIOD_TICKS;
    TIM3->ARR = SERVO_PERIOD_TICKS;
    TIM4->ARR = SERVO_PERIOD_TICKS;

    Set_Arm(90);
    Set_Top(90);
    Set_Base(90);
    Set_Gripper(90);
}

void Set_Arm(int deg)
{
    uint32_t ticks;
    uint32_t ticks_mirror;

    if (deg < 0)
    {
        deg = 0;
    }
    if (deg > ARM_MAX_DEG)
    {
        deg = ARM_MAX_DEG;
    }
    arm_deg = deg;

    ticks = angle_to_ticks(deg, ARM_MAX_DEG);
    ticks_mirror = (SERVO_MIN_US + SERVO_MAX_US) - ticks;

    TIM4->CCR3 = ticks;
    TIM1->CCR4 = ticks_mirror;
}

void Set_Top(int deg)
{
    if (deg < 0)
    {
        deg = 0;
    }
    if (deg > TOP_MAX_DEG)
    {
        deg = TOP_MAX_DEG;
    }
    top_deg = deg;

    TIM3->CCR4 = angle_to_ticks(deg, TOP_MAX_DEG);
}

void Set_Base(int deg)
{
    if (deg < 0)
    {
        deg = 0;
    }
    if (deg > BASE_MAX_DEG)
    {
        deg = BASE_MAX_DEG;
    }
    base_deg = deg;

    TIM4->CCR1 = angle_to_ticks(deg, BASE_MAX_DEG);
}

void Set_Gripper(int deg)
{
    if ((deg < GRIP_MIN_DEG) || (deg > GRIP_MAX_DEG))
    {
        printf("ERROR: Gripper angle %d out of range [%d, %d]\r\n",
               deg, GRIP_MIN_DEG, GRIP_MAX_DEG);
        return;
    }

    gripper_deg = deg;
    TIM1->CCR3 = angle_to_ticks(deg, GRIP_PHYSICAL_MAX_DEG);
}

int Get_Arm(void)
{
    return arm_deg;
}

int Get_Top(void)
{
    return top_deg;
}

int Get_Base(void)
{
    return base_deg;
}

int Get_Gripper(void)
{
    return gripper_deg;
}
