#include "../include/ble_protocal.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(blec, LOG_LEVEL_INF);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
static volatile bool device_connected = false;
static void start_advertising();

static const struct bt_data ad[] = {
  // BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
  // BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xde, 0xfa), // X16-Service
  // BT_DATA_BYTES(BT_DATA_SVC_DATA16, 0xde, 0xfa, 'n', 'o', 'r', 'd', 'i', 'c') // nordic version.
  
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
            BT_UUID_16_ENCODE(0xfaca),  // FACA
					  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),  // BAT
					  BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)), // DIS
	// BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
};

// Set Scan Response data
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	int err;
	struct bt_conn_info info;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		LOG_INF("Connection failed (err %d)\n", conn_err);
		return;
	}
  if (!device_connected) {
		device_connected = true;
    // device_connected = (NULL != bt_conn_ref(conn));
    // stop adv
    ble_stop_adv();
  }
  // dis service
	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_INF("Failed to get connection info (err %d)\n", err);
	} else {
		const struct bt_conn_le_phy_info *phy_info;
		phy_info = info.le.phy;

		LOG_INF("Connected: %s, tx_phy %u, rx_phy %u\n",
		       addr, phy_info->tx_phy, phy_info->rx_phy);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t err)
{
	LOG_INF("Disconnected (reason 0x%02x)\n", err);
  if (err) {
  } else {
  }
  if (device_connected) {
		device_connected = false;
    // bt_conn_unref(conn);
    // start adv again.
    ble_start_adv();
  }
}

// define callback
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void start_advertising()
{
	int err;
	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_INF("Failed to start advertising set (err %d)\n", err);
		return;
	}
	LOG_INF("Advertiser started\n");
}

int ble_start_adv()
{	
  start_advertising();
  return 0;
}

int ble_stop_adv()
{
  return bt_le_adv_stop();
}

int ble_init_adv()
{
  int err = 0;
  return err;
}
