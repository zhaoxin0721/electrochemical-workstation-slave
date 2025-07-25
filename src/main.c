#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <unistd.h> // For sleep function
#include <stdlib.h> // For EXIT_FAILURE and EXIT_SUCCESS

#include "../include/ble_protocal.h"
#include "../include/ble_faca.h"
#include "../drivers/ad5940_nrf52.h"
#include "../drivers/dac.h"// Include DAC driver header


LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {.att_mtu_updated = mtu_updated};

void main(void)
{
  int err = 0;
  // AD5940
  err = ad5940_spi_init();
  if (err)
  {
    LOG_ERR("AD5940 SPI init failed (err %d)", err);
    return;
  }

  err = ad5940_enable_int();
  if (err)
  {
    LOG_ERR("AD5940 INT init failed (err %d)", err);
    return;
  }
  LOG_INF("AD5940 init success.");

  // Bluetooth
  err = bt_enable(NULL);
  if (err)
  {
    LOG_ERR("Bluetooth init failed (err %d)", err);
    return;
  }
  bt_gatt_cb_register(&gatt_callbacks);
  LOG_INF("Bluetooth init success.");
  if (IS_ENABLED(CONFIG_BT_SETTINGS))
  {
    settings_load();
    LOG_INF("Settings init success.");
  }
  err = ble_init_adv();
  if (err)
  {
    LOG_ERR("Bluetooth adv init failed (err %d)", err);
    return;
  }
  LOG_INF("Bluetooth adv init success.");
  err = ble_start_adv();
  if (err)
  {
    LOG_ERR("Bluetooth adv start failed (err %d)", err);
    return;
  }
  LOG_INF("Bluetooth adv start success.");

  err = ble_faca_init();
  if (err)
  {
    LOG_ERR("Bluetooth faca init failed (err %d)", err);
    return;
  }
  LOG_INF("Bluetooth faca init success.");
  
  for(;;) {
    k_msleep(1);
  }
}

//adc
#define ADC_CHANNEL 1
void adc_task(void)
{
    if (adc_init() != 0) {
        printf("ADC初始化失败！\n");
        return;
    }
    printf("ADC初始化成功，开始读取数据...\n");

    // 循环读取ADC值
    while (1) {
        // 读取原始ADC值
        uint16_t adc_value = adc_read(ADC_CHANNEL);

        // 检查读取是否成功（假设返回0xFFFF表示失败）
        if (adc_value == 0xFFFF) {
            printf("读取ADC通道 %d 失败\n", ADC_CHANNEL);
        } else {
            // 转换为电压值
            float result = *adc_read(ADC_CHANNEL);

            // 打印结果
            printf("ADC通道 %d: 原始值 = %d, 电压 = %.2fV\n",
                   ADC_CHANNEL, adc_value, result);
        }

        // 每隔1秒读取一次
        sleep(1);
    }
    // 释放ADC资源（实际中可能不会执行到这里）
    adc_deinit();
}
