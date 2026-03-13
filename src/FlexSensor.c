#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <BOARD.h>
#include <Timers.h>
#include <Pwm.h>
#include <Adc.h>
#include <FlexSensor.h>

static bool k_isCurled = false;

void FLEX_Init(void)
{
    ADC_Init(ADC_SINGLE_SHOT_POLLING);
}

bool FLEX_isFingerCurled(void)
{
    float reading = FLEX_getReading();

    if (!k_isCurled) {//if straight
        k_isCurled = (FLEX_ADC_CLOSE > reading);//stays straight until goes below close thresh
    } else {//if curled
        k_isCurled = (FLEX_ADC_OPEN < reading);//stays curled until goes above open thresh
    }
    return k_isCurled;
}

float FLEX_getReading(void)
{
    return ADC_Read(FLEX_ADC_CHANNEL);
}

//remove this eventually
float map_flex_to_angle(uint16_t adc)
{
    float ratio;

    if (FLEX_ADC_OPEN > FLEX_ADC_CLOSE) {
        if (adc >= FLEX_ADC_OPEN) {
            ratio = 0.0f;
        } else if (adc <= FLEX_ADC_CLOSE) {
            ratio = 1.0f;
        } else {
            ratio = (float)(FLEX_ADC_OPEN - adc) / (float)(FLEX_ADC_OPEN - FLEX_ADC_CLOSE);
        }
    } else if (FLEX_ADC_OPEN < FLEX_ADC_CLOSE) {
        if (adc <= FLEX_ADC_OPEN) {
            ratio = 0.0f;
        } else if (adc >= FLEX_ADC_CLOSE) {
            ratio = 1.0f;
        } else {
            ratio = (float)(adc - FLEX_ADC_OPEN) / (float)(FLEX_ADC_CLOSE - FLEX_ADC_OPEN);
        }
    } else {
        ratio = 0.0f;
    }

    return GRIPPER_OPEN_DEG + ratio * (GRIPPER_CLOSE_DEG - GRIPPER_OPEN_DEG);
}
