/**
 * ChronoAmperometric
 * 
 * Updated: 2023-11-21, 初版
*/

#include "common.h"
#include <stdbool.h>

typedef struct {
  float     amplitude;
  uint32_t  ms;
} elec_ca_config_t;

int elec_ca_init();

int elec_ca_config(elec_ca_config_t config_);

int elec_ca_start();

int elec_ca_stop();

bool elec_ca_is_working();
