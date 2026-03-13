/**
 * @file    IMU.h
 * @brief   Simple BNO055 tilt interface for baseline-relative pitch and roll.
 *
 * Call IMU_Init() after BOARD_Init() and Timers_Init(). The module captures the
 * current hand pose as neutral and reports pitch/roll deviation in degrees.
 */

#ifndef IMU_H
#define IMU_H

/** @brief Initialize the BNO055 and capture the current pose as the baseline. */
void IMU_Init(void);

/** @brief Return roll deviation from the saved baseline in degrees. */
float IMU_GetRollAngle(void);

/** @brief Return pitch deviation from the saved baseline in degrees. */
float IMU_GetPitchAngle(void);

/** @brief Re-capture the current pose as the new zero reference. */
void IMU_ResetBaseline(void);

#endif /* IMU_H */
