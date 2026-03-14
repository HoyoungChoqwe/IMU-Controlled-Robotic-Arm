#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <BNO055.h>
#include <BOARD.h>

#include "IMU.h"
#include "robotCommon.h"

/* Baseline capture and filtering parameters copied from imuarmcode. */
#define BASELINE_SAMPLES 50U
#define BASELINE_SAMPLE_DELAY_MS 10U
#define FILTER_ALPHA 0.20f
#define RAD_TO_DEG 57.2957795f
#define MIN_VECTOR_MAGNITUDE 0.0001f

/* Steepness thresholds in degrees with hysteresis between adjacent levels. */
#define LEVEL_0_TO_1_ENTER_DEG 15.0f
#define LEVEL_1_TO_0_EXIT_DEG 8.0f
#define LEVEL_1_TO_2_ENTER_DEG 30.0f
#define LEVEL_2_TO_1_EXIT_DEG 23.0f
#define LEVEL_2_TO_3_ENTER_DEG 45.0f
#define LEVEL_3_TO_2_EXIT_DEG 38.0f

/* Default neutral orientation used before the sensor baseline is captured. */
#define DEFAULT_GRAVITY_VEC {0.0f, 0.0f, 1.0f}
#define DEFAULT_ANGLE_DEG 0.0f

typedef struct {
    float x;
    float y;
    float z;
} Vec3;

static const Vec3 k_defaultGravity = DEFAULT_GRAVITY_VEC;

static Vec3 k_filteredAccel = DEFAULT_GRAVITY_VEC;
static Vec3 k_currentGravity = DEFAULT_GRAVITY_VEC;
static Vec3 k_baselineGravity = DEFAULT_GRAVITY_VEC;
static float k_baselineRollDeg = DEFAULT_ANGLE_DEG;
static float k_baselinePitchDeg = DEFAULT_ANGLE_DEG;
static int8_t k_rollRangeLevel = 0;
static int8_t k_pitchRangeLevel = 0;
static uint8_t k_isInitialized = FALSE;

/* Add two 3-axis vectors component-by-component. */
static Vec3 vec3_add(Vec3 a, Vec3 b)
{
    Vec3 result = {a.x + b.x, a.y + b.y, a.z + b.z};
    return result;
}

/* Subtract one 3-axis vector from another. */
static Vec3 vec3_sub(Vec3 a, Vec3 b)
{
    Vec3 result = {a.x - b.x, a.y - b.y, a.z - b.z};
    return result;
}

/* Scale a vector by a single float multiplier. */
static Vec3 vec3_scale(Vec3 value, float scale)
{
    Vec3 result = {value.x * scale, value.y * scale, value.z * scale};
    return result;
}

/* Compute vector length for normalization and zero-vector checks. */
static float vec3_magnitude(Vec3 value)
{
    return sqrtf((value.x * value.x) + (value.y * value.y) + (value.z * value.z));
}

/* Convert a raw vector into unit length while keeping a safe fallback direction. */
static Vec3 vec3_normalize(Vec3 value, Vec3 fallback)
{
    float magnitude = vec3_magnitude(value);

    if (magnitude < MIN_VECTOR_MAGNITUDE) {
        return fallback;
    }

    /* Scale by the reciprocal so the normalized vector ends up with magnitude 1. */
    return vec3_scale(value, 1.0f / magnitude);
}

/* Read the current raw accelerometer sample from the BNO055. */
static Vec3 read_accel_vector(void)
{
    Vec3 accel = {
        (float) BNO055_ReadAccelX(),
        (float) BNO055_ReadAccelY(),
        (float) BNO055_ReadAccelZ()
    };
    return accel;
}

/* Pitch is tilt forward/backward relative to the gravity-aligned Z axis. */
static float compute_pitch_deg(Vec3 gravity)
{
    return atan2f(gravity.x, gravity.z) * RAD_TO_DEG;
}

/* Roll is tilt side-to-side relative to the gravity-aligned Z axis. */
static float compute_roll_deg(Vec3 gravity)
{
    return atan2f(gravity.y, gravity.z) * RAD_TO_DEG;
}

/* Average several accel readings to establish a stable neutral gravity vector. */
static Vec3 capture_baseline_gravity(void)
{
    Vec3 total = {DEFAULT_ANGLE_DEG, DEFAULT_ANGLE_DEG, DEFAULT_ANGLE_DEG};
    Vec3 fallback = k_defaultGravity;

    /* Start at sample 0 and accumulate exactly BASELINE_SAMPLES readings. */
    for (uint32_t sample = 0; sample < BASELINE_SAMPLES; sample++)
    {
        total = vec3_add(total, read_accel_vector());
        MAGIC_delayms(BASELINE_SAMPLE_DELAY_MS);
    }

    return vec3_normalize(total, fallback);
}

/* Low-pass filter live accel data into a smoother gravity estimate for angle reads. */
static void update_gravity(void)
{
    Vec3 rawAccel;

    if (!k_isInitialized) {
        return;
    }

    rawAccel = read_accel_vector();
    k_filteredAccel = vec3_add(
        k_filteredAccel,
        vec3_scale(vec3_sub(rawAccel, k_filteredAccel), FILTER_ALPHA)
    );
    k_currentGravity = vec3_normalize(k_filteredAccel, k_baselineGravity);
}

/*
 * Apply the requested hysteresis bands to one signed angle value.
 * A sign change must re-enter through level +/-1 after crossing +/-15 degrees.
 */
static int8_t update_range_level(float angleDeg, int8_t currentLevel)
{
    if (currentLevel == 0) {
        if (angleDeg > LEVEL_0_TO_1_ENTER_DEG) {
            return 1;
        }
        if (angleDeg < -LEVEL_0_TO_1_ENTER_DEG) {
            return -1;
        }
        return 0;
    }

    if (currentLevel > 0) {
        if (angleDeg < -LEVEL_0_TO_1_ENTER_DEG) {
            return -1;
        }

        if (currentLevel == 1) {
            if (angleDeg < LEVEL_1_TO_0_EXIT_DEG) {
                return 0;
            }
            if (angleDeg > LEVEL_1_TO_2_ENTER_DEG) {
                return 2;
            }
            return 1;
        }

        if (currentLevel == 2) {
            if (angleDeg < LEVEL_2_TO_1_EXIT_DEG) {
                return 1;
            }
            if (angleDeg > LEVEL_2_TO_3_ENTER_DEG) {
                return 3;
            }
            return 2;
        }

        if (angleDeg < LEVEL_3_TO_2_EXIT_DEG) {
            return 2;
        }
        return 3;
    }

    if (angleDeg > LEVEL_0_TO_1_ENTER_DEG) {
        return 1;
    }

    if (currentLevel == -1) {
        if (angleDeg > -LEVEL_1_TO_0_EXIT_DEG) {
            return 0;
        }
        if (angleDeg < -LEVEL_1_TO_2_ENTER_DEG) {
            return -2;
        }
        return -1;
    }

    if (currentLevel == -2) {
        if (angleDeg > -LEVEL_2_TO_1_EXIT_DEG) {
            return -1;
        }
        if (angleDeg < -LEVEL_2_TO_3_ENTER_DEG) {
            return -3;
        }
        return -2;
    }

    if (angleDeg > -LEVEL_3_TO_2_EXIT_DEG) {
        return -2;
    }
    return -3;
}

void IMU_Init(void)
{
    if (BNO055_Init() != SUCCESS) {
        MAGIC_display_error_oled("ERROR: BNO055_Init\r\n", ERROR_SHOW_TIME);
        printf("ERROR: BNO055_Init\r\n");
        while (1);
    }

    k_isInitialized = TRUE;
    IMU_ResetBaseline();
}

void IMU_ResetBaseline(void)
{
    if (!k_isInitialized) {
        return;
    }

    k_baselineGravity = capture_baseline_gravity();
    k_filteredAccel = k_baselineGravity;
    k_currentGravity = k_baselineGravity;
    k_baselinePitchDeg = compute_pitch_deg(k_baselineGravity);
    k_baselineRollDeg = compute_roll_deg(k_baselineGravity);
    k_rollRangeLevel = 0;
    k_pitchRangeLevel = 0;
}

float IMU_GetRollAngle(void)
{
    if (!k_isInitialized) {
        return DEFAULT_ANGLE_DEG;
    }

    update_gravity();
    return compute_roll_deg(k_currentGravity) - k_baselineRollDeg;
}

float IMU_GetPitchAngle(void)
{
    if (!k_isInitialized) {
        return DEFAULT_ANGLE_DEG;
    }

    update_gravity();
    return compute_pitch_deg(k_currentGravity) - k_baselinePitchDeg;
}

int8_t IMU_GetRollRange(void)
{
    float rollAngleDeg;

    if (!k_isInitialized) {
        return 0;
    }

    rollAngleDeg = IMU_GetRollAngle();
    k_rollRangeLevel = update_range_level(rollAngleDeg, k_rollRangeLevel);
    return k_rollRangeLevel;
}

int8_t IMU_GetPitchRange(void)
{
    float pitchAngleDeg;

    if (!k_isInitialized) {
        return 0;
    }

    pitchAngleDeg = IMU_GetPitchAngle();
    k_pitchRangeLevel = update_range_level(pitchAngleDeg, k_pitchRangeLevel);
    return k_pitchRangeLevel;
}
