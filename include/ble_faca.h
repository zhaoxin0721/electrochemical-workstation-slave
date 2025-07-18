#include <stdint.h>

#define BLE_CMD_BLUFI_SSIDPSWD    0x01
#define BLE_CMD_BLUFI_DISCONNECT  0x02
#define BLE_CMD_SESSION_START     0x03
#define BLE_CMD_SESSION_TS        0x04
#define BLE_CMD_SESSION_STOP      0x05
#define BLE_CMD_GET_WIFI_CONFIG   0x06
#define BLE_CMD_GET_TASK_CONFIG   0x07
#define BLE_CMD_POST_TASK_CONFIG  0x08
#define BLE_CMD_READ_REGISTER     0x09
#define BLE_CMD_WRITE_REGISTER    0x0A

int ble_faca_init();
