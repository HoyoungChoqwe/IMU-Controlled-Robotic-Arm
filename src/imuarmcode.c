#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <BNO055.h>
#include <BOARD.h>
#include <Timers.h>

#include "Servo.h"

#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2

#define BASELINE_SAMPLES 50U
#define BASELINE_SAMPLE_DELAY_US 10000U
#define GYRO_BIAS_SAMPLES 400U
#define GYRO_BIAS_SAMPLE_DELAY_US 5000U
#define PRINT_PERIOD_US 100000U
#define FILTER_ALPHA 0.20f
#define RAD_TO_DEG 57.2957795f
#define GYRO_LSB_PER_DPS 16.0f
#define GYRO_DEADBAND_DPS 1.25f
#define AUTO_RECENTER_STILL_US 1500000U
#define AUTO_RECENTER_GYRO_DPS 3.0f
#define AUTO_RECENTER_GRAVITY_DELTA 0.020f

#define SERVO_UPDATE_PERIOD_US 20000U

#define BASE_MAX_DEG 180.0f
#define BASE_CENTER_DEG 90.0f
#define BASE_SLOW_STEP_DEG 0.8f
#define BASE_FAST_STEP_DEG 2.2f

#define ARM_MAX_DEG 180.0f
#define ARM_CENTER_DEG 90.0f
#define ARM_SLOW_STEP_DEG 0.7f
#define ARM_FAST_STEP_DEG 1.8f

#define LEFT_MAX_READING 300.0f
#define RIGHT_MIN_READING -450.0f
#define UP_MAX_READING 350.0f
#define DOWN_MIN_READING -250.0f
#define SPEED_1_RATIO 0.35f
#define SPEED_2_RATIO 0.70f

/*
 * left/right uses the gyro-based turn angle around gravity.
 * Flip the sign if the direction is reversed.
 * Scale it if you want a more or less aggressive output.
 */
#define LEFT_RIGHT_SIGN 1.0f
#define LEFT_RIGHT_SCALE 1.0f

/*
 * up/down uses the tilt response that was previously showing up
 * in the old left/right accelerometer path: tilt axis versus normal axis.
 * Change the axis pair or scale if needed.
 */
#define UP_DOWN_TILT_AXIS AXIS_X
#define UP_DOWN_NORMAL_AXIS AXIS_Z
#define UP_DOWN_SIGN 1.0f
#define UP_DOWN_SCALE 5.0f

typedef struct
{
    float x;
    float y;
    float z;
} Vec3;

static void DelayUs(uint32_t delayUs)
{
    uint32_t startUs = Timers_GetMicroSeconds();
    while ((uint32_t)(Timers_GetMicroSeconds() - startUs) < delayUs)
    {
    }
}

static int UserButtonPressed(void)
{
    return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);
}

static Vec3 Vec3Add(Vec3 a, Vec3 b)
{
    Vec3 result = {a.x + b.x, a.y + b.y, a.z + b.z};
    return result;
}

static Vec3 Vec3Sub(Vec3 a, Vec3 b)
{
    Vec3 result = {a.x - b.x, a.y - b.y, a.z - b.z};
    return result;
}

static Vec3 Vec3Scale(Vec3 value, float scale)
{
    Vec3 result = {value.x * scale, value.y * scale, value.z * scale};
    return result;
}

static float ClampFloat(float value, float minValue, float maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }
    if (value > maxValue)
    {
        return maxValue;
    }
    return value;
}

static float Vec3Dot(Vec3 a, Vec3 b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

static float Vec3Magnitude(Vec3 value)
{
    return sqrtf((value.x * value.x) + (value.y * value.y) + (value.z * value.z));
}

static Vec3 Vec3Normalize(Vec3 value, Vec3 fallback)
{
    float magnitude = Vec3Magnitude(value);

    if (magnitude < 0.0001f)
    {
        return fallback;
    }

    return Vec3Scale(value, 1.0f / magnitude);
}

static Vec3 ReadAccelVector(void)
{
    Vec3 accel = {
        (float) BNO055_ReadAccelX(),
        (float) BNO055_ReadAccelY(),
        (float) BNO055_ReadAccelZ()
    };
    return accel;
}

static Vec3 ReadGyroVectorRaw(void)
{
    Vec3 gyro = {
        (float) BNO055_ReadGyroX(),
        (float) BNO055_ReadGyroY(),
        (float) BNO055_ReadGyroZ()
    };
    return gyro;
}

static float GetAxisValue(Vec3 value, int axisId)
{
    if (axisId == AXIS_X)
    {
        return value.x;
    }
    if (axisId == AXIS_Y)
    {
        return value.y;
    }
    return value.z;
}

static float ComputeUpDownAngle(Vec3 gravity)
{
    float tiltAxis = GetAxisValue(gravity, UP_DOWN_TILT_AXIS);
    float normalAxis = GetAxisValue(gravity, UP_DOWN_NORMAL_AXIS);

    return atan2f(tiltAxis, normalAxis) * RAD_TO_DEG;
}

static int SpeedCommandFromReading(float reading, float positiveMax, float negativeMin)
{
    float positiveSlow = positiveMax * SPEED_1_RATIO;
    float positiveFast = positiveMax * SPEED_2_RATIO;
    float negativeSlow = negativeMin * SPEED_1_RATIO;
    float negativeFast = negativeMin * SPEED_2_RATIO;

    if (reading >= positiveFast)
    {
        return 2;
    }
    if (reading >= positiveSlow)
    {
        return 1;
    }
    if (reading <= negativeFast)
    {
        return -2;
    }
    if (reading <= negativeSlow)
    {
        return -1;
    }
    return 0;
}

static float NormalizedReadingMagnitude(float reading, float positiveMax, float negativeMin)
{
    float magnitude;

    if (reading > 0.0f)
    {
        magnitude = reading / positiveMax;
    }
    else if (reading < 0.0f)
    {
        magnitude = reading / negativeMin;
    }
    else
    {
        magnitude = 0.0f;
    }

    return ClampFloat(magnitude, 0.0f, 1.0f);
}

static float StepSizeForCommand(int command, float slowStepDeg, float fastStepDeg)
{
    if ((command == 2) || (command == -2))
    {
        return fastStepDeg;
    }
    if ((command == 1) || (command == -1))
    {
        return slowStepDeg;
    }
    return 0.0f;
}

static void ResetControlledServosToInitPose(float *baseAngleDeg, float *armAngleDeg)
{
    *baseAngleDeg = BASE_CENTER_DEG;
    *armAngleDeg = ARM_CENTER_DEG;

    Set_Base((int) *baseAngleDeg);
    Set_Arm((int) *armAngleDeg);
}

static Vec3 CaptureBaselineGravity(void)
{
    Vec3 total = {0.0f, 0.0f, 0.0f};
    Vec3 fallback = {0.0f, 0.0f, 1.0f};

    for (uint32_t sample = 0; sample < BASELINE_SAMPLES; sample++)
    {
        total = Vec3Add(total, ReadAccelVector());
        DelayUs(BASELINE_SAMPLE_DELAY_US);
    }

    return Vec3Normalize(total, fallback);
}

static Vec3 CalibrateGyroBiasLsb(void)
{
    Vec3 total = {0.0f, 0.0f, 0.0f};

    for (uint32_t sample = 0; sample < GYRO_BIAS_SAMPLES; sample++)
    {
        total = Vec3Add(total, ReadGyroVectorRaw());
        DelayUs(GYRO_BIAS_SAMPLE_DELAY_US);
    }

    return Vec3Scale(total, 1.0f / (float) GYRO_BIAS_SAMPLES);
}

static void CaptureNeutralPose(Vec3 *baselineGravity, Vec3 *gyroBiasLsb, float *leftRightDeg, float *baselineUpDownDeg)
{
    printf("Hold your hand still in the neutral pose.\r\n");
    DelayUs(1000000U);

    *baselineGravity = CaptureBaselineGravity();
    *gyroBiasLsb = CalibrateGyroBiasLsb();
    *leftRightDeg = 0.0f;
    *baselineUpDownDeg = ComputeUpDownAngle(*baselineGravity);

    printf("Neutral pose captured.\r\n");
}

int main(void)
{
    Vec3 baselineGravity;
    Vec3 filteredAccel;
    Vec3 gyroBiasLsb;
    Vec3 currentGravity;
    Vec3 previousGravity;
    float baselineUpDownDeg;
    float leftRightDeg = 0.0f;
    float upDownDeg = 0.0f;
    float baseAngleDeg = BASE_CENTER_DEG;
    float armAngleDeg = ARM_CENTER_DEG;
    uint32_t previousSampleUs;
    uint32_t lastPrintUs;
    uint32_t lastServoUpdateUs;
    uint32_t lastMotionUs;
    float gyroMagnitudeDps = 0.0f;
    uint8_t autoRecenterArmed = TRUE;

    if (BNO055_Init() != SUCCESS)
    {
        printf("BNO055 init failed. Check power, SDA, SCL, and address.\r\n");
        while (TRUE)
        {
        }
    }

    Servo_Init();
    ResetControlledServosToInitPose(&baseAngleDeg, &armAngleDeg);

    printf("IMU test ready.\r\n");
    printf("left/right = gyro-based turn angle around gravity\r\n");
    printf("up/down = accelerometer tilt from the X/Z path\r\n");
    printf("base servo = PWM_4, arm servos = PWM_3/PWM_5 mirrored\r\n");
    printf("Press the blue USER button while neutral to re-zero IMU.\r\n");

    CaptureNeutralPose(&baselineGravity, &gyroBiasLsb, &leftRightDeg, &baselineUpDownDeg);
    filteredAccel = baselineGravity;
    currentGravity = baselineGravity;
    previousGravity = baselineGravity;
    previousSampleUs = Timers_GetMicroSeconds();
    lastPrintUs = previousSampleUs;
    lastServoUpdateUs = previousSampleUs;
    lastMotionUs = previousSampleUs;

    while (TRUE)
    {
        uint32_t currentUs = Timers_GetMicroSeconds();
        uint32_t deltaUs = (uint32_t)(currentUs - previousSampleUs);

        if (UserButtonPressed())
        {
            ResetControlledServosToInitPose(&baseAngleDeg, &armAngleDeg);
            CaptureNeutralPose(&baselineGravity, &gyroBiasLsb, &leftRightDeg, &baselineUpDownDeg);
            filteredAccel = baselineGravity;
            currentGravity = baselineGravity;
            previousGravity = baselineGravity;
            previousSampleUs = Timers_GetMicroSeconds();
            lastPrintUs = previousSampleUs;
            lastServoUpdateUs = previousSampleUs;
            lastMotionUs = previousSampleUs;
            autoRecenterArmed = TRUE;
            DelayUs(250000U);
            continue;
        }

        if (deltaUs == 0U)
        {
            continue;
        }
        previousSampleUs = currentUs;

        {
            Vec3 rawAccel = ReadAccelVector();
            Vec3 rawGyro = ReadGyroVectorRaw();
            Vec3 gyroDps;
            float gyroRateDps;
            float gravityDelta;

            filteredAccel = Vec3Add(
                filteredAccel,
                Vec3Scale(Vec3Sub(rawAccel, filteredAccel), FILTER_ALPHA)
            );
            currentGravity = Vec3Normalize(filteredAccel, baselineGravity);
            gyroDps = Vec3Scale(Vec3Sub(rawGyro, gyroBiasLsb), 1.0f / GYRO_LSB_PER_DPS);
            gyroMagnitudeDps = Vec3Magnitude(gyroDps);
            gyroRateDps = Vec3Dot(gyroDps, currentGravity) * LEFT_RIGHT_SIGN * LEFT_RIGHT_SCALE;

            if (fabsf(gyroRateDps) < GYRO_DEADBAND_DPS)
            {
                gyroRateDps = 0.0f;
            }

            leftRightDeg += gyroRateDps * (((float) deltaUs) / 1000000.0f);
            gravityDelta = Vec3Magnitude(Vec3Sub(currentGravity, previousGravity));

            if ((gyroMagnitudeDps > AUTO_RECENTER_GYRO_DPS) || (gravityDelta > AUTO_RECENTER_GRAVITY_DELTA))
            {
                lastMotionUs = currentUs;
                autoRecenterArmed = TRUE;
            }
            else if (autoRecenterArmed
                    && ((uint32_t)(currentUs - lastMotionUs) >= AUTO_RECENTER_STILL_US))
            {
                float recenteredLeftRightDeg = leftRightDeg;
                float recenteredUpDownDeg = upDownDeg;

                baselineGravity = currentGravity;
                baselineUpDownDeg = ComputeUpDownAngle(currentGravity);
                leftRightDeg = 0.0f;
                upDownDeg = 0.0f;
                autoRecenterArmed = FALSE;
                printf("\r\nAUTO RECENTERED after 1.5 s stillness: lr=%+.2f up=%+.2f -> zero\r\n",
                       recenteredLeftRightDeg,
                       recenteredUpDownDeg);
            }

            previousGravity = currentGravity;
        }
        upDownDeg = (ComputeUpDownAngle(currentGravity) - baselineUpDownDeg)
                * UP_DOWN_SIGN * UP_DOWN_SCALE;

        if ((uint32_t)(currentUs - lastServoUpdateUs) >= SERVO_UPDATE_PERIOD_US)
        {
            int baseCommand = SpeedCommandFromReading(leftRightDeg, LEFT_MAX_READING, RIGHT_MIN_READING);
            int armCommand = SpeedCommandFromReading(upDownDeg, UP_MAX_READING, DOWN_MIN_READING);
            float baseMagnitude = NormalizedReadingMagnitude(leftRightDeg, LEFT_MAX_READING, RIGHT_MIN_READING);
            float armMagnitude = NormalizedReadingMagnitude(upDownDeg, UP_MAX_READING, DOWN_MIN_READING);
            float baseStep = StepSizeForCommand(baseCommand, BASE_SLOW_STEP_DEG, BASE_FAST_STEP_DEG);
            float armStep = StepSizeForCommand(armCommand, ARM_SLOW_STEP_DEG, ARM_FAST_STEP_DEG);

            if (baseMagnitude >= armMagnitude)
            {
                armCommand = 0;
                armStep = 0.0f;
            }
            else
            {
                baseCommand = 0;
                baseStep = 0.0f;
            }

            if (baseCommand > 0)
            {
                baseAngleDeg += baseStep;
            }
            else if (baseCommand < 0)
            {
                baseAngleDeg -= baseStep;
            }

            if (armCommand > 0)
            {
                armAngleDeg += armStep;
            }
            else if (armCommand < 0)
            {
                armAngleDeg -= armStep;
            }

            baseAngleDeg = ClampFloat(baseAngleDeg, 0.0f, BASE_MAX_DEG);
            armAngleDeg = ClampFloat(armAngleDeg, 0.0f, ARM_MAX_DEG);

            Set_Base((int) baseAngleDeg);
            Set_Arm((int) armAngleDeg);
            lastServoUpdateUs = currentUs;
        }

        if ((uint32_t)(currentUs - lastPrintUs) >= PRINT_PERIOD_US)
        {
            int baseCommand = SpeedCommandFromReading(leftRightDeg, LEFT_MAX_READING, RIGHT_MIN_READING);
            int armCommand = SpeedCommandFromReading(upDownDeg, UP_MAX_READING, DOWN_MIN_READING);
            float baseMagnitude = NormalizedReadingMagnitude(leftRightDeg, LEFT_MAX_READING, RIGHT_MIN_READING);
            float armMagnitude = NormalizedReadingMagnitude(upDownDeg, UP_MAX_READING, DOWN_MIN_READING);
            const char *activeAxis;

            if (baseMagnitude >= armMagnitude)
            {
                armCommand = 0;
            }
            else
            {
                baseCommand = 0;
            }

            if ((baseCommand == 0) && (armCommand == 0))
            {
                activeAxis = "none";
            }
            else if (baseCommand != 0)
            {
                activeAxis = "base";
            }
            else
            {
                activeAxis = "arm";
            }

            lastPrintUs = currentUs;
            printf("left/right=%+7.2f   up/down=%+7.2f   active=%s   baseSpeed=%+d   armSpeed=%+d   base=%.1f   arm=%.1f\r\n",
                   leftRightDeg,
                   upDownDeg,
                   activeAxis,
                   baseCommand,
                   armCommand,
                   baseAngleDeg,
                   armAngleDeg);
        }
    }
}
