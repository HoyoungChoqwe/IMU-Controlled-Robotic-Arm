#ifndef SERVO_H
#define SERVO_H

/**
 * Initialize PWM hardware and move all servos to 90 degrees.
 * Must be called after BOARD_Init() and Timers_Init().
 */
void Servo_Init(void);

/**
 * Set arm angle in degrees, clamped to [0, 180].
 * The mirrored arm servo is updated automatically.
 */
void Set_Arm(int deg);

/**
 * Set top angle in degrees, clamped to [0, 180].
 * Top servo output: PWM_6 / PC9 / TIM3_CH4.
 */
void Set_Top(int deg);

/**
 * Set base angle in degrees, clamped to [0, 180].
 */
void Set_Base(int deg);

/**
 * Set gripper angle in degrees.
 * Valid range is [35, 205].
 */
void Set_Gripper(int deg);

int Get_Arm(void);
int Get_Top(void);
int Get_Base(void);
int Get_Gripper(void);

#endif
