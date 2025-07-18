#ifndef ENABLE_FACA_SERVICE__
#define ENABLE_FACA_SERVICE__

#include "../include/ble_faca.h"

#include <stdlib.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(faca, LOG_LEVEL_INF);

// Electrochemical Methods
#include "../elec_methods/common.h"
#include "../elec_methods/ocp.h"
#include "../elec_methods/eis.h"
#include "../elec_methods/cv.h"
#include "../elec_methods/ca.h"


// Looper

#define LOOPER_INTERVAL         1     // 1ms

static void looper_work_handler(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(looper_work, looper_work_handler);

// Task
static volatile uint8_t current_elec_method = 0x00; // undefined.
static inline void reset_current_elec_method() { current_elec_method = 0x00; }

#define BLE_BUFFER_MAX_SIZE 20
static          uint8_t buffer_copi[BLE_BUFFER_MAX_SIZE];
static volatile uint8_t buffer_size = BLE_BUFFER_MAX_SIZE;
static volatile bool    buffer_done = false;

// Bluetooth

#define FACA_INDEX        0
#define FAC1_INDEX        2
#define FAC2_INDEX        4
#define FAC2_CCC_INDEX    5
#define FAC3_INDEX        7
#define FAC3_CCC_INDEX    8

static uint8_t response_values[500];
static volatile bool notify_fac2_enable;
static volatile bool notify_fac3_enable;
static void mpu_ccc_cfg_changed_fac2(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	notify_fac2_enable = (value == BT_GATT_CCC_NOTIFY);
	// LOG_INF("Notification %s", notify_enable ? "enabled" : "disabled");
}
static void mpu_ccc_cfg_changed_fac3(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	notify_fac3_enable = (value == BT_GATT_CCC_NOTIFY);
  // LOG_INF("notify_fac3_enable: %d", notify_fac3_enable ? 1 : 0);
}
static ssize_t read_faca(struct bt_conn *conn, const struct bt_gatt_attr *attr,
   void *buf, uint16_t len, uint16_t offset);
static ssize_t on_request_arrived(struct bt_conn *conn,
		    const struct bt_gatt_attr *attr, const void *buf,
		    uint16_t len, uint16_t offset, uint8_t flags);

static struct bt_uuid_128 faca_service_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x0000faca, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));

// FAC1: recv payload from master.
static struct bt_uuid_128 fac1_char_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x0000fac1, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));

// FAC2: notify normal callback.
static struct bt_uuid_128 fac2_char_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x0000fac2, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));

// FAC3: notify adc data. "0000fac3-0000-1000-8000-00805f9b34fb"
static struct bt_uuid_128 fac3_char_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x0000fac3, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));

BT_GATT_SERVICE_DEFINE(faca_service,
  BT_GATT_PRIMARY_SERVICE(&faca_service_uuid),
  BT_GATT_CHARACTERISTIC(&fac1_char_uuid.uuid,
            BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
            BT_GATT_PERM_WRITE, NULL, on_request_arrived, (void *)1),
  BT_GATT_CHARACTERISTIC(&fac2_char_uuid.uuid, BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_faca, NULL, &response_values),
  // BT_GATT_CUD("Fade2 CUD", BT_GATT_PERM_READ),
  BT_GATT_CCC(mpu_ccc_cfg_changed_fac2, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
  BT_GATT_CHARACTERISTIC(&fac3_char_uuid.uuid, BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_faca, NULL, &buffer_copi),
  // BT_GATT_CUD("Fade3 CUD", BT_GATT_PERM_READ),
  BT_GATT_CCC(mpu_ccc_cfg_changed_fac3, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static ssize_t read_faca(struct bt_conn *conn, const struct bt_gatt_attr *attr,
   void *buf, uint16_t len, uint16_t offset)
{
  const char *value = attr->user_data;
  return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value));
}

///////////////////////////////////////////// FAC2 Reply //////////////////////////////////////////////////

// static void reply_response_with_data(uint16_t request_id, uint8_t status, uint8_t * data, uint8_t len)
// {
//   response_values[0] = (uint8_t)(request_id >> 8) & 0xFF;
//   response_values[1] = (request_id & 0xFF);
//   response_values[2] = status;
//   for (uint8_t i = 0; i < len; i++) {
//     response_values[i + 3] = data[i];
//   }
//   uint8_t err = bt_gatt_notify(NULL, &faca_service.attrs[FAC2_INDEX], &response_values, len + 3);
//   if (err) {
//     // ...
//     // LOG_ERR("faca2 reply json error, err = %02x", err);
//   }
// }
static void reply_response(uint16_t request_id, uint8_t status)
{
  response_values[0] = (uint8_t)((request_id >> 8) & 0xFF);
  response_values[1] = (request_id & 0xFF);
  response_values[2] = status;
  uint8_t err = bt_gatt_notify(NULL, &faca_service.attrs[FAC2_INDEX], &response_values, 3);
  if (err) {
    // ...
    // LOG_ERR("faca2 reply json error, err = %02x", err);
  }
}

///////////////////////////////////////////// FAC3 Notify //////////////////////////////////////////////////

// worker, implement from common.h
void send_fac_stream(uint8_t * data, uint8_t len)
{
  buffer_copi[0] = current_elec_method;
  // copy buffer
  for (uint8_t i = 0; i < len; i++) {
    buffer_copi[i + 1] = data[i];
  }
  buffer_size = len + 1;
  // signal flag
  buffer_done = true;
}

int send_fac_stream_directly(uint8_t * data, uint8_t len)
{
  uint8_t buffer_[len];
  buffer_[0] = current_elec_method;
  for (int i = 0; i < len; i++) {
    buffer_[i + 1] = data[i];
  }
  int err = bt_gatt_notify(NULL, &faca_service.attrs[FAC3_INDEX], &buffer_, (len + 1));
  if (err) {
    return err;
  }
  return 0;
}


#define COMMIT_BUFFER_MAX_LEN 4096
#define COMMIT_BUFFER_EACH_LEN 20
static volatile uint32_t c_buffer_size;
static volatile uint32_t c_buffer_index; // for sender process.
static uint8_t c_buffer[COMMIT_BUFFER_MAX_LEN];
void commit_fac_stream(uint8_t * data, uint8_t len, bool append)
{
  // buffer here.
  int index = 0;
  if (append) {
    index = c_buffer_size;
  } else {
    index = 0;
    c_buffer_index = 0;
  }
  for(int i = 0; i < len; i++) {
    c_buffer[index++] = data[i];
  }
  c_buffer_size = index;
}


static inline void stop_other_elec_methods(uint8_t except_code) {
  if (except_code != CODE_OCP) {
    if (elec_ocp_is_working()) elec_ocp_stop();
  }
  if (except_code != CODE_EIS) {
    if (elec_eis_is_working()) elec_eis_stop();
  }
  if (except_code != CODE_CV) {
    if (elec_cv_is_working()) elec_cv_stop();
  }
  k_msleep(500); // 延迟 500ms 确保 AD5940 能正确重置
  // reset ad5940
  // ...
}


static ssize_t on_request_arrived(struct bt_conn *conn,
		    const struct bt_gatt_attr *attr, const void *buf,
		    uint16_t len, uint16_t offset, uint8_t flags)
{
  // parse json.
  LOG_INF("payload len: %d", len);
  if (len == 1) { // debug mode
    int err = 0; // ad594x_verify();
    LOG_INF("faca: ad594x debug verify: %02x", err);
    return 0;
  }
  /**
   * 
   * Request Payload Parser & Resp.
   * 
  */
  uint8_t * data = (uint8_t *)buf;
  uint16_t request_id = (uint16_t)(data[0] << 8) | (data[1] & 0xFF);
  uint8_t method = data[2] & 0xFF;
  switch(method) {
  case 0x01: // 计时电流法
  {
    break;
  }
  case CODE_OCP: // OCP
  {
    uint8_t code = data[3];
    if (code == 0x01) { // start cmd
      LOG_INF("Router: OCP/start");
      if (elec_ocp_is_working()) {
        reply_response(request_id, 1);
      } else {
        stop_other_elec_methods(method);
        elec_ocp_init();
        reply_response(request_id, 0);
        current_elec_method = method;
        // start work
        elec_ocp_start();
        // elec_ocp_start(&send_fac_stream);
      }
      break;
    }
    if (code == 0x02) { // stop cmd
      LOG_INF("Router: OCP/stop");
      elec_ocp_stop();
      reply_response(request_id, 0);
      reset_current_elec_method(); // reset
      break;
    }
    if (code == 0x03) { // config
      LOG_INF("Router: OCP/config");
      opc_config_t config;
      config.sample_frequency = data[4]; // positive integer, range in 1..50
      if (len >= 7) {
        config.delay = (data[5] << 8) | data[6];
      }
      elec_ocp_config(config);
      reply_response(request_id, 0);
      break;
    }
    break;
  }
  case CODE_EIS: // EIS
  {
    uint8_t code = data[3];
    if (code == 0x01) { // start cmd
      LOG_INF("Router: EIS/start");
      if (elec_eis_is_working()) {
        reply_response(request_id, 1);
      } else {
        stop_other_elec_methods(method);
        reply_response(request_id, 0);
        LOG_INF("EIS init#begin");
        elec_eis_init(); // will take a lot of time.
        current_elec_method = method;
        // start work
        LOG_INF("EIS start#begin");
        elec_eis_start();
        LOG_INF("EIS start#end");
      }
      break;
    }
    if (code == 0x02) { // stop cmd
      LOG_INF("Router: EIS/stop");
      elec_eis_stop();
      reply_response(request_id, 0);
      reset_current_elec_method(); // reset
      break;
    }
    if (code == 0x03) { // config cmd
      LOG_INF("Router: EIS/config");
      if (len < 19) {
        reply_response(request_id, 1);
        break;
      }
      uint32_t freq_begin = ((uint32_t)data[4]) << 24;
      freq_begin |= ((uint32_t)data[5]) << 16;
      freq_begin |= ((uint32_t)data[6]) << 8;
      freq_begin |= data[7] & 0xFF;

      uint32_t freq_end = ((uint32_t)data[8]) << 24;
      freq_end |= ((uint32_t)data[9]) << 16;
      freq_end |= ((uint32_t)data[10]) << 8;
      freq_end |= data[11] & 0xFF;

      uint16_t sample_count = ((uint32_t)data[12]) << 8;
      sample_count |= data[13] & 0xFF;

      bool use_log = data[14] > 0;

      int16_t ac_volt = ((uint32_t)data[15]) << 24;
      ac_volt |= ((uint32_t)data[16]) << 16;
      ac_volt |= ((uint32_t)data[17]) << 8;
      ac_volt |= data[18] & 0xFF;

      elec_eis_config_t config;
      config.freq_begin = freq_begin / 100.0F;
      config.freq_end   = freq_end / 100.0F;
      config.sample_count = sample_count;
      config.use_log    = use_log;
      config.ac_volt    = ac_volt / 100.0F;

      LOG_INF("=======eis config updated========");
      LOG_INF("freq begin:    %f Hz", config.freq_begin);
      LOG_INF("freq end:      %f Hz", config.freq_end);
      LOG_INF("sample count:  %d", config.sample_count);
      LOG_INF("use_log:       %s", config.use_log ? "yes" : "no");
      LOG_INF("ac volt:       %f mV", config.ac_volt);
      LOG_INF("=======eis config updated========");

      elec_eis_config(config);
      reply_response(request_id, 0);
      break;
    }
    break;
  }
  case CODE_CV: // CV
  {
    uint8_t code = data[3];
    if (code == 0x01) { // start cmd
      LOG_INF("Router: CV/start");
      if (elec_cv_is_working()) {
        reply_response(request_id, 1);
      } else {
        stop_other_elec_methods(method);
        elec_cv_init();
        reply_response(request_id, 0);
        current_elec_method = method;
        // start work
        elec_cv_start();
      }
      break;
    }
    if (code == 0x02) { // stop cmd
      LOG_INF("Router: CV/stop");
      elec_cv_stop();
      reply_response(request_id, 0);
      reset_current_elec_method(); // reset
      break;
    }
    if (code == 0x03) { // config cmd
      LOG_INF("Router: CV/config");
      if (len < 26) { // valid len: 6*int + 1*short = 26
        reply_response(request_id, 1);
        break;
      }
      int k = 4;
      elec_cv_config_t config;

      uint8_t rtia = data[k++];
      if (rtia > 0 && rtia <= 26) {
        config.rtia = rtia;
      }

      int32_t vertex_begin = ((uint32_t)data[k++]) << 24;
      vertex_begin |= ((uint32_t)data[k++]) << 16;
      vertex_begin |= ((uint32_t)data[k++]) << 8;
      vertex_begin |= data[k++] & 0xFF;
      config.vertex_begin = vertex_begin / 100.0f; // mV

      int32_t vertex_1 = ((uint32_t)data[k++]) << 24;
      vertex_1 |= ((uint32_t)data[k++]) << 16;
      vertex_1 |= ((uint32_t)data[k++]) << 8;
      vertex_1 |= data[k++] & 0xFF;
      config.vertex_1 = vertex_1 / 100.0f; // mV

      int32_t vertex_2 = ((uint32_t)data[k++]) << 24;
      vertex_2 |= ((uint32_t)data[k++]) << 16;
      vertex_2 |= ((uint32_t)data[k++]) << 8;
      vertex_2 |= data[k++] & 0xFF;
      config.vertex_2 = vertex_2 / 100.0f; // mV

      int32_t sample_delay = ((uint32_t)data[k++]) << 24;
      sample_delay |= ((uint32_t)data[k++]) << 16;
      sample_delay |= ((uint32_t)data[k++]) << 8;
      sample_delay |= data[k++] & 0xFF;
      config.sample_delay = sample_delay / 100.0f; // ms

      int32_t scan_rate = ((uint32_t)data[k++]) << 24;
      scan_rate |= ((uint32_t)data[k++]) << 16;
      scan_rate |= ((uint32_t)data[k++]) << 8;
      scan_rate |= data[k++] & 0xFF;
      config.scan_rate = scan_rate / 100.0f; // mV/s

      int32_t step_value = ((uint32_t)data[k++]) << 24;
      step_value |= ((uint32_t)data[k++]) << 16;
      step_value |= ((uint32_t)data[k++]) << 8;
      step_value |= data[k++] & 0xFF;
      config.step_value = step_value / 100.0f; // mV

      uint16_t cycle_number = ((uint16_t)data[k++]) << 8;
      cycle_number |= data[k++] & 0xFF;
      config.cycle_number = cycle_number;

      LOG_INF("=======cv config updated========");
      LOG_INF("begin volt:       %f mV", config.vertex_begin);
      LOG_INF("vertex1 volt:     %f mV", config.vertex_1);
      LOG_INF("vertex2 start:    %f mV", config.vertex_2);
      LOG_INF("step:             %f mV", config.step_value);
      LOG_INF("scan rate:        %f mV/s", config.scan_rate);
      LOG_INF("sample delay:     %f ms", config.sample_delay);
      LOG_INF("cycle number:     %d",    config.cycle_number);
      LOG_INF("r tia:            %d",    config.rtia);
      LOG_INF("=======cv config updated========");
      elec_cv_config(config);

      reply_response(request_id, 0);
      break;
    }
    break;
  }
  case CODE_CA: // CA
  {
    uint8_t code = data[3];
    if (code == 0x01) { // start cmd
      LOG_INF("Router: CA/start");
      if (elec_ca_is_working()) {
        reply_response(request_id, 1);
      } else {
        stop_other_elec_methods(method);
        elec_ca_init();
        reply_response(request_id, 0);
        current_elec_method = method;
        // start work
        elec_ca_start();
      }
      break;
    }
    if (code == 0x02) { // stop cmd
      LOG_INF("Router: CA/stop");
      elec_ca_stop();
      reply_response(request_id, 0);
      reset_current_elec_method(); // reset
      break;
    }
    if (code == 0x03) { // config cmd
      LOG_INF("Router: CA/config");
      if (len < 12) { // valid len: 10
        reply_response(request_id, 1);
        break;
      }
      int k = 4;
      elec_ca_config_t config;
      int32_t amp = ((uint32_t)data[k++]) << 24;
      amp |= ((uint32_t)data[k++]) << 16;
      amp |= ((uint32_t)data[k++]) << 8;
      amp |= data[k++] & 0xFF;

      uint16_t ms = ((uint16_t)data[k++]) << 24;
      ms |= ((uint32_t)data[k++]) << 16;
      ms |= ((uint32_t)data[k++]) << 8;
      ms |= data[k++] & 0xFF;

      config.amplitude  = amp / 100.0f;  // mV
      config.ms    = ms;   // ms

      LOG_INF("=======ca config updated========");
      LOG_INF("start volt:    %f mV", config.amplitude);
      LOG_INF("peak volt:     %d ms", config.ms);
      LOG_INF("=======ca config updated========");
      elec_ca_config(config);

      reply_response(request_id, 0);
      break;
    }
    break;
  }
  default: break;
  }
	return 0;
}


static void looper_work_handler(struct k_work *work)
{
  // if (!notify_fac3_enable) return;
  if (buffer_done) 
  {
    buffer_done = false;
    // notify
    uint8_t err = bt_gatt_notify(NULL, &faca_service.attrs[FAC3_INDEX], &buffer_copi, buffer_size);
    if (err) {
      // ...
      // retry--
      err = bt_gatt_notify(NULL, &faca_service.attrs[FAC3_INDEX], &buffer_copi, buffer_size);
    }
  }
  int max_len = c_buffer_size - c_buffer_index;
  if (max_len > 1) {
    int len = max_len > COMMIT_BUFFER_EACH_LEN ? COMMIT_BUFFER_EACH_LEN : max_len;
    uint8_t buffer_c_copi[len];
    for (int i = 0; i < len; i++) {
      buffer_c_copi[i] = c_buffer[c_buffer_index + i];
    }
    int err = bt_gatt_notify(NULL, &faca_service.attrs[FAC3_INDEX], &buffer_c_copi, len);
    if (err) {
      // retry
    } else {
      c_buffer_index += len;
    }
  }
	k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(LOOPER_INTERVAL));
}

int ble_faca_init()
{
  int err = 0;
  err = k_work_schedule(&looper_work, K_NO_WAIT);
  return 0;
}

#endif
