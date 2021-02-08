#define __DATA_LOCK process = 1
#define __DATA_UNLOCK process = 0
#define __DATA_LOCKED process == 1

#define FFT_DATA_SIZE 1024
#define ADC_DATA_SIZE FFT_DATA_SIZE*2
#define ADC_RESOLUTION 4096

#define PWM_PERIOD 255
#define LIGHT_LEVEL 0.5
#define FADE_PERIOD 1000
