#ifndef NRF52_PERIPHERAL_H
#define NRF52_PERIPHERAL_H

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>

// 只声明，不实现
int AD5940_ReadADC(void);

#endif // NRF52_PERIPHERAL_H
