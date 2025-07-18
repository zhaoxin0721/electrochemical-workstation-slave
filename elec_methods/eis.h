/**
 * EIS 电化学阻抗测量
 * 
 * AIN2 + SE0，不区分正负
 * 
 * Updated: 2023-11-21, 初版
*/

#include "common.h"
#include <stdbool.h>

typedef struct {
  float freq_begin;
  float freq_end;
  uint16_t sample_count;
  float ac_volt; // 300mV defalut.
  bool  use_log;
} elec_eis_config_t;

int elec_eis_init();

int elec_eis_config(elec_eis_config_t config_);

int elec_eis_start();

int elec_eis_stop();

bool elec_eis_is_working();
