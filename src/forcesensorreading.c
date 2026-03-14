#include <stdint.h>
#include <stdio.h>

#include <Adc.h>
#include <BOARD.h>
#include <Timers.h>

/*
 * Change this to the ADC channel your force sensor is wired to.
 * Available choices on this board setup are:
 *   ADC_0, ADC_1, ADC_2, ADC_3, ADC_4, ADC_5, POT
 */
#define FORCE_SENSOR_CHANNEL ADC_0

#define PRINT_PERIOD_MS 100U
#define FORCE_SAMPLES_PER_PRINT 8U
#define ADC_REF_VOLTAGE 3.3f
#define FORCE_RAW_MIN 0.0f
#define FORCE_RAW_MAX 4095.0f

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

static uint16_t ReadForceSensorRaw(void)
{
    uint32_t total = 0U;

    for (uint32_t sample = 0; sample < FORCE_SAMPLES_PER_PRINT; sample++)
    {
        total += ADC_Read(FORCE_SENSOR_CHANNEL);
    }

    return (uint16_t) (total / FORCE_SAMPLES_PER_PRINT);
}

static float RawToVoltage(uint16_t raw)
{
    return (((float) raw) / FORCE_RAW_MAX) * ADC_REF_VOLTAGE;
}

static float RawToPercent(uint16_t raw)
{
    float normalized = ((((float) raw) - FORCE_RAW_MIN) / (FORCE_RAW_MAX - FORCE_RAW_MIN)) * 100.0f;

    return ClampFloat(normalized, 0.0f, 100.0f);
}

int main(void)
{
    uint32_t lastPrintMs;

    BOARD_Init();
    Timers_Init();

    if (ADC_Init(ADC_SINGLE_SHOT_POLLING) != SUCCESS)
    {
        printf("ADC init failed.\r\n");
        while (TRUE)
        {
        }
    }

    printf("Force sensor test ready.\r\n");
    printf("Reading FORCE_SENSOR_CHANNEL in src/main.c.\r\n");
    printf("If nothing changes, switch FORCE_SENSOR_CHANNEL to ADC_1/ADC_2/.../ADC_5 or POT.\r\n");

    lastPrintMs = Timers_GetMilliSeconds();

    while (TRUE)
    {
        uint32_t currentMs = Timers_GetMilliSeconds();

        if ((uint32_t) (currentMs - lastPrintMs) >= PRINT_PERIOD_MS)
        {
            uint16_t raw = ReadForceSensorRaw();
            float voltage = RawToVoltage(raw);
            float percent = RawToPercent(raw);

            lastPrintMs = currentMs;
            printf("force raw=%4u   voltage=%1.3f V   level=%5.1f%%\r\n",
                   raw,
                   voltage,
                   percent);
        }
    }
}
