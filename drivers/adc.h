#include <stdint.h>
#include <stdbool.h>

#define ENABLE_NRF_ADC

#define CHANNEL_0_7
// #define CHANNEL_0_3
#if !defined(CHANNEL_16_23)
// 因为 Channel 4-7 
#define ENABLE_BATTERY_ADC
#endif

typedef enum {
#if defined(CHANNEL_0_3)
  ADC_CH_0 = 0x00,
  ADC_CH_1,
  ADC_CH_2,
  ADC_CH_3,
  
#endif
#if defined(CHANNEL_4_7)
  ADC_CH_4,
  ADC_CH_5,
  ADC_CH_6,
  ADC_CH_7,

#endif
  ADC_CH_SIZE,
} adc_channel_t;

int adcx_init();

int32_t adcx_channel_read(adc_channel_t channel);

int32_t adcx_status(bool status);

#if defined(ENABLE_BATTERY_ADC)
int32_t adcx_channel_read_battery();
#endif
