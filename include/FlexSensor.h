
// Gripper motion limits (tune as needed)
#define GRIPPER_OPEN_DEG   0.0f
#define GRIPPER_CLOSE_DEG  135.0f

// Flex calibration (your latest values, reversed direction)
// 1700 -> open, 600 -> close
#define FLEX_ADC_OPEN      1100U//1700U
#define FLEX_ADC_CLOSE      500U//600U

#define FLEX_ADC_CHANNEL   ADC_0
#define CONTROL_PERIOD_MS  20U
#define PRINT_PERIOD_MS   200U

void FLEX_Init(void);
bool FLEX_isFingerCurled(void);
float FLEX_getReading(void);
float map_flex_to_angle(uint16_t adc);
