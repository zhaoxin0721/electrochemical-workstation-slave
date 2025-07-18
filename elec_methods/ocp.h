/**
 * OCP 电势测量
 * 
 * 正：AIN1
 * 负：VBAIS_CAP
 * 
 * Updated: 2023-11-03, 初版
*/

#include "common.h"
#include <stdbool.h>

typedef struct {
  uint16_t sample_frequency; // default
  uint16_t delay;
} opc_config_t;

int elec_ocp_config(opc_config_t config);

void elec_ocp_init(void);

int elec_ocp_start();
// int elec_ocp_start(send_fac_stream_t send_fac_stream);

int elec_ocp_stop();

bool elec_ocp_is_working();
