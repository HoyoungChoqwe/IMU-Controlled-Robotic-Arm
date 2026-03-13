/**
 * @file    Servo.h
 * @brief   Unified servo control for ECE167 robot arm.
 *
 * Pin assignments (UCSC Nucleo I/O Shield):
 *   Gripper : Pin 55  PWM2  PA10  TIM1_CH3   TD8120  35-205 deg (hard limits)
 *   Base    : Pin 57  PWM4  PB6   TIM4_CH1   TD8120  0-180 deg  (180-deg version)
 *   Arm     : Pin 58  PWM5  PB8   TIM4_CH3   TD8120  0-180 deg  (primary)
 *             Pin 56  PWM3  PA11  TIM1_CH4              (mirror, back-to-back mount)
 */

#ifndef SERVO_H
#define SERVO_H

/**
 * @brief  Initialize PWM hardware and move all servos to 90 degrees.
 *         Must be called after BOARD_Init() and Timers_Init().
 */
void Servo_Init(void);

/**
 * @brief  Set arm angle (0-180 degrees).
 *         The mirrored servo is driven automatically.
 */
void Set_Arm(int deg);

/**
 * @brief  Set base angle (0-270 degrees).
 */
void Set_Base(int deg);

/**
 * @brief  Set gripper angle (35-205 degrees).
 *         Prints error and does NOT move if deg is outside [35, 205].
 */
void Set_Gripper(int deg);

/** @brief  Return current arm angle in degrees. */
int Get_Arm(void);

/** @brief  Return current base angle in degrees (0-180). */
int Get_Base(void);

/** @brief  Return current gripper angle in degrees. */
int Get_Gripper(void);

#endif /* SERVO_H */
