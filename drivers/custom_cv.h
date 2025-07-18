
#include "ad594x.h"

typedef struct {
  float start_volt;   // -1000.0f              /* -1V */
  float peak_volt;    // +1000.0f              /* +1V */
  float vzero_start;  // = 1300.0f;            /* 1.3V */
  float vzero_peak;   // = 1300.0f;            /* 1.3V */
  uint32_t duration;  // = 24*1000;            /* X * 1000, where x is total duration of ramp signal. Unit is ms. */
  uint16_t sample_delay; // = 7.0f;               /* 7ms. Time between update DAC and ADC sample. Unit is ms. */
  uint16_t step_number;  // = 800;              /* Total steps. Equals to ADC sample number */
  uint16_t repeat_number; // 重复次数
} ccv_params_t;

typedef void (* ccv_callback_t)(float volt_mV, float current_uA);

int ccv_init(ccv_params_t params);

int ccv_start(ccv_callback_t ccv_callback);
