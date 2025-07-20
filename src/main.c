#include <zephyr/kernel.h>

#include <zephyr/sys/util.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>

#include "../include/ble_protocal.h"
#include "../include/ble_faca.h"
#include "../drivers/nrf52_peripheral.h" //将#include "../drivers/ad5940_nrf52.h" 改为#include "../drivers/nrf52_peripheral.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {.att_mtu_updated = mtu_updated};

void main(void)
{
  int err = 0;
  // AD5940改成nrf52832
  err = nrf52_spi_init();
  if (err)
  {
    LOG_ERR("nrf52 SPI init failed (err %d)", err);
    return;
  }

  err = nrf52_enable_int();
  if (err)
  {
    LOG_ERR("nrf52 INT init failed (err %d)", err);
    return;
  }
  LOG_INF("nrf52 init success.");

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
  
  return;
}
