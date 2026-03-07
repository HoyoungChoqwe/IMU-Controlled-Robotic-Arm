// Hold all defines for hardware specs in one place easily edited
#pragma once

// Servo timing constants (1MHz timer clock -> 1 tick = 1us)
#define SERVO_PERIOD_TICKS  19999   // ARR for 50Hz: 1,000,000/50 - 1 = 19999
#define SERVO_MIN_US        500     // 0.5ms pulse = 0 degrees   (270-deg version)
#define SERVO_MAX_US        2500    // 2.5ms pulse = 270 degrees (270-deg version)
#define SERVO_MAX_DEG       270.0f
#define SERVO_MIN_DEG       0.0f

// Step size and delay per step for the sweep
#define SWEEP_STEP_DEG      1       // 1 degree per step
#define SWEEP_STEP_DELAY_MS 20      // 20ms per step (~5.4 sec for full 0-270 sweep)
