/**
 * Cyclic Voltammetry 循环伏安法
 * 
 * Connect each resistor network pin to the CE0, RE0, SE0, and DE0 pins on the P5 header.
 * 
 * If the definition of OPT_RAMP_MEAS (parameter defined in the x.h file) is set to 1,the following four measurements are performed:
 *   Current through SE0.
     Voltage on SE0.
     Voltage on RE0.
     Current through SE0 measured a second time.
 * 
 * Updated: 2023-11-22, 初版
*/

#include "common.h"
#include <stdbool.h>

typedef struct {
  float step_value;       // = <100mV;
  float scan_rate;        // = 50mV/s;
  float sample_delay;     // = 7.0f;
  float vertex_begin;     // -1000.0f              /* -1V */
  float vertex_1;         // +1000.0f              /* +1V */
  float vertex_2;         // = 1300.0f;            /* 1.3V */
  uint16_t cycle_number;  // 重复次数
  uint8_t rtia;           // 1-26
} elec_cv_config_t;

int elec_cv_init();

int elec_cv_config(elec_cv_config_t config_);

int elec_cv_start();

int elec_cv_stop();

bool elec_cv_is_working();
