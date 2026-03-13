// /*
//  * FLEX -> GRIPPER (single-servo only)
//  * Board: Nucleo F411RE + UCSC IO Shield
//  * Servo: PWM2 = TIM1_CH3 = PA10
//  * Flex sensor: ADC_0 = PA0 (Analog0)
//  */
//
// #include <stdio.h>
// #include <BOARD.h>
// #include <Timers.h>
// #include <Pwm.h>
// #include <Adc.h>
//
// #define GRIPPER_PWM_CHANNEL PWM_2
//
// // Servo timing (1 MHz timer tick => 1 tick = 1 us)
// #define SERVO_PERIOD_TICKS 19999U    // 50 Hz
// #define SERVO_MIN_US       500.0f
// #define SERVO_MAX_US       2500.0f
// #define SERVO_MAX_DEG      180.0f
//
// // Gripper motion limits (tune as needed)
// #define GRIPPER_OPEN_DEG   0.0f
// #define GRIPPER_CLOSE_DEG  135.0f
//
// // Flex calibration (your latest values, reversed direction)
// // 1700 -> open, 600 -> close
// #define FLEX_ADC_OPEN      1700U
// #define FLEX_ADC_CLOSE      600U
//
// #define FLEX_ADC_CHANNEL   ADC_0
// #define CONTROL_PERIOD_MS  20U
// #define PRINT_PERIOD_MS   200U
//
// static void delay_ms(uint32_t ms)
// {
//     uint32_t start = Timers_GetMilliSeconds();
//     while ((Timers_GetMilliSeconds() - start) < ms) {
//     }
// }
//
// static float map_flex_to_angle(uint16_t adc)
// {
//     float ratio;
//
//     // Handle reversed calibration cleanly.
//     if (FLEX_ADC_OPEN > FLEX_ADC_CLOSE) {
//         if (adc >= FLEX_ADC_OPEN) {
//             ratio = 0.0f;
//         } else if (adc <= FLEX_ADC_CLOSE) {
//             ratio = 1.0f;
//         } else {
//             ratio = (float)(FLEX_ADC_OPEN - adc) / (float)(FLEX_ADC_OPEN - FLEX_ADC_CLOSE);
//         }
//     } else if (FLEX_ADC_OPEN < FLEX_ADC_CLOSE) {
//         if (adc <= FLEX_ADC_OPEN) {
//             ratio = 0.0f;
//         } else if (adc >= FLEX_ADC_CLOSE) {
//             ratio = 1.0f;
//         } else {
//             ratio = (float)(adc - FLEX_ADC_OPEN) / (float)(FLEX_ADC_CLOSE - FLEX_ADC_OPEN);
//         }
//     } else {
//         ratio = 0.0f;
//     }
//
//     return GRIPPER_OPEN_DEG + ratio * (GRIPPER_CLOSE_DEG - GRIPPER_OPEN_DEG);
// }
//
// static void gripper_set_angle(float angle_deg)
// {
//     if (angle_deg < 0.0f) {
//         angle_deg = 0.0f;
//     }
//     if (angle_deg > SERVO_MAX_DEG) {
//         angle_deg = SERVO_MAX_DEG;
//     }
//
//     uint32_t ticks = (uint32_t)(SERVO_MIN_US +
//                                 (angle_deg / SERVO_MAX_DEG) * (SERVO_MAX_US - SERVO_MIN_US));
//     TIM1->CCR3 = ticks;
// }
//
// int main(void)
// {
//     uint32_t last_print_ms;
//
//     BOARD_Init();
//     Timers_Init();
//
//     printf("Flex -> Gripper (PWM2/PA10 only)\r\n");
//
//     if (PWM_Init() == ERROR) {
//         printf("ERROR: PWM_Init failed\r\n");
//         while (1) {
//         }
//     }
//     if (PWM_AddPin(GRIPPER_PWM_CHANNEL) == ERROR) {
//         printf("ERROR: PWM_AddPin(PWM_2) failed\r\n");
//         while (1) {
//         }
//     }
//     if (ADC_Init(ADC_SINGLE_SHOT_POLLING) == ERROR) {
//         printf("ERROR: ADC_Init failed\r\n");
//         while (1) {
//         }
//     }
//
//     // Force 50 Hz frame for servo control.
//     TIM1->ARR = SERVO_PERIOD_TICKS;
//
//     last_print_ms = Timers_GetMilliSeconds();
//
//     while (1) {
//         uint16_t adc_value = ADC_Read(FLEX_ADC_CHANNEL);
//         float angle = map_flex_to_angle(adc_value);
//
//         gripper_set_angle(angle);
//
//         if ((Timers_GetMilliSeconds() - last_print_ms) >= PRINT_PERIOD_MS) {
//             last_print_ms = Timers_GetMilliSeconds();
//             printf("ADC=%u angle=%.1f\r\n", adc_value, angle);
//         }
//
//         delay_ms(CONTROL_PERIOD_MS);
//     }
//
//     return 0;
// }
