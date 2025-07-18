#include "common.h"
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(elec_common, LOG_LEVEL_INF);

static volatile loop_method_t current_loop_method = NULL;

int elec_method_commit(loop_method_t loop_method)
{
  current_loop_method = loop_method;
  return 0;
}

void elec_loop(void)
{
  while(true) {
    // select method
    if (current_loop_method != NULL) {
      LOG_INF("Method Running...");
      current_loop_method();
      // (*current_loop_method)();
    }
    k_msleep(1); // 1ms delay.
  }
}

K_THREAD_DEFINE(elec_looper, STACKSIZE, elec_loop, NULL, NULL, NULL, PRIORITY, 0, 0);
