#include "adc.h"
#ifdef ENABLE_NRF_ADC

#if defined(CHANNEL_16_23)
#define ENABLE_MULTPLEX3
#endif

#include <nrfx_saadc.h>
#include <nrfx_gpiote.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adcx, LOG_LEVEL_INF);

#define SAADC_PRIORITY      7           // SAADC中断优先级
#if defined(CHANNEL_16_23)       // 如果需要支持16-23通道，定义ENABLE_MULTPLEX3（启用第3个多路复用器）
#define ENABLE_MULTPLEX3
#endif

#if defined(ENABLE_BATTERY_ADC) // 动态定义 ADC 通道数组的大小 1.如果启用电池ADC
  #if defined(ENABLE_MULTPLEX3) // 2.如果启用多路复用3
    #define SAADC_CHANNEL_SIZE  4
  #else
    #define SAADC_CHANNEL_SIZE  3
  #endif
#else
  #if defined(ENABLE_MULTPLEX3)
    #define SAADC_CHANNEL_SIZE  3
  #else
    #define SAADC_CHANNEL_SIZE  2
  #endif
#endif
static nrf_saadc_value_t adc_value[SAADC_CHANNEL_SIZE];
// static volatile bool is_ready = true;

#define MULTIPX_A1     NRF_GPIO_PIN_MAP(0, 00)
#define MULTIPX_B1     NRF_GPIO_PIN_MAP(0, 26)
#define MULTIPX_C1     NRF_GPIO_PIN_MAP(0, 04)

// #define MULTIPX_A2     NRF_GPIO_PIN_MAP(0, 13)
// #define MULTIPX_B2     NRF_GPIO_PIN_MAP(0, 15)
// #define MULTIPX_C2     NRF_GPIO_PIN_MAP(0, 17)
#define MULTIPX_A2     NRF_GPIO_PIN_MAP(0, 22)
#define MULTIPX_B2     NRF_GPIO_PIN_MAP(0, 24)
#define MULTIPX_C2     NRF_GPIO_PIN_MAP(1, 0)

#if defined(ENABLE_MULTPLEX3)
#define MULTIPX_A3     NRF_GPIO_PIN_MAP(0, 17)
#define MULTIPX_B3     NRF_GPIO_PIN_MAP(0, 15)
#define MULTIPX_C3     NRF_GPIO_PIN_MAP(0, 13)
#endif

#define __wait                for(int t = 0; t < 1000; t++) { __NOP(); } // k_usleep(2);

static volatile bool adc_inited = false;

int adcx_init()
{
  if (adc_inited) return 0;
  int channel_index = -1;
  nrfx_saadc_channel_t adc_channel0    = NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN4, ++channel_index);
  adc_channel0.channel_config.acq_time = NRF_SAADC_ACQTIME_10US;
  adc_channel0.channel_config.gain     = NRF_SAADC_GAIN1_6;
  nrfx_saadc_channel_t adc_channel1    = NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN6, ++channel_index);
  adc_channel1.channel_config.acq_time = NRF_SAADC_ACQTIME_10US;
  adc_channel1.channel_config.gain     = NRF_SAADC_GAIN1_6;
#if defined(ENABLE_MULTPLEX3)
  nrfx_saadc_channel_t adc_channel2    = NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN2, ++channel_index);
  adc_channel2.channel_config.acq_time = NRF_SAADC_ACQTIME_10US;
  adc_channel2.channel_config.gain     = NRF_SAADC_GAIN1_6;
#endif
  nrfx_saadc_channel_t adc_channels[SAADC_CHANNEL_SIZE] = {
    adc_channel0,
    adc_channel1,
#if defined(ENABLE_MULTPLEX3)
    adc_channel2,
#endif
  };
  // 初始化SAADC外设（传入中断优先级）
  nrfx_err_t err = nrfx_saadc_init(SAADC_PRIORITY);
  if (err != NRFX_SUCCESS) { LOG_ERR("adc init error: %d", err); return err; }
  err = nrfx_saadc_channels_config(&adc_channels, SAADC_CHANNEL_SIZE);

  uint32_t mask = 0b11;
#if defined(ENABLE_MULTPLEX3)
  mask = (mask << 1) | 0x01;
#endif
    // 设置SAADC工作模式
  err = nrfx_saadc_simple_mode_set((mask),
                                        NRF_SAADC_RESOLUTION_14BIT,
                                        NRF_SAADC_OVERSAMPLE_DISABLED,
                                        NULL);
                                        // saadc_event_handler);
  if (err != NRFX_SUCCESS) LOG_ERR("adc mode set error: %08X", err);
// 设置ADC缓冲区
  err = nrfx_saadc_buffer_set(adc_value, SAADC_CHANNEL_SIZE);
  if (err != NRFX_SUCCESS) LOG_ERR("adc buffer set error: %08X", err);

  // gpio
  nrfx_gpiote_output_config_t a1 = NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
  a1.drive = NRF_GPIO_PIN_H0H1;
  nrfx_gpiote_output_configure(MULTIPX_A1, &a1, NULL);
  nrfx_gpiote_output_config_t b1 = NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
  b1.drive = NRF_GPIO_PIN_H0H1;
  nrfx_gpiote_output_configure(MULTIPX_B1, &b1, NULL);
  nrfx_gpiote_output_config_t c1 = NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
  c1.drive = NRF_GPIO_PIN_H0H1;
  nrfx_gpiote_output_configure(MULTIPX_C1, &c1, NULL);
  nrfx_gpiote_output_config_t a2 = NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
  a2.drive = NRF_GPIO_PIN_H0H1;
  nrfx_gpiote_output_configure(MULTIPX_A2, &a2, NULL);
  nrfx_gpiote_output_config_t b2 = NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
  b2.drive = NRF_GPIO_PIN_H0H1;
  nrfx_gpiote_output_configure(MULTIPX_B2, &b2, NULL);
  nrfx_gpiote_output_config_t c2 = NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
  c2.drive = NRF_GPIO_PIN_H0H1;
  nrfx_gpiote_output_configure(MULTIPX_C2, &c2, NULL);
#if defined(ENABLE_MULTPLEX3)
  nrfx_gpiote_output_config_t a3 = NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
  a3.drive = NRF_GPIO_PIN_H0H1;
  nrfx_gpiote_output_configure(MULTIPX_A3, &a3, NULL);
  nrfx_gpiote_output_config_t b3 = NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
  b3.drive = NRF_GPIO_PIN_H0H1;
  nrfx_gpiote_output_configure(MULTIPX_B3, &b3, NULL);
  nrfx_gpiote_output_config_t c3 = NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
  c3.drive = NRF_GPIO_PIN_H0H1;
  nrfx_gpiote_output_configure(MULTIPX_C3, &c3, NULL);
#endif

  nrfx_gpiote_input_config_t in1 = NRFX_GPIOTE_DEFAULT_INPUT_CONFIG;
  nrfx_gpiote_input_configure(NRF_GPIO_PIN_MAP(0, 03), &in1, NULL, NULL);
  nrfx_gpiote_input_config_t in2 = NRFX_GPIOTE_DEFAULT_INPUT_CONFIG;
  nrfx_gpiote_input_configure(NRF_GPIO_PIN_MAP(0, 00), &in2, NULL, NULL);
  nrfx_gpiote_out_clear(MULTIPX_A1);
  nrfx_gpiote_out_clear(MULTIPX_B1);
  nrfx_gpiote_out_clear(MULTIPX_A2);
  nrfx_gpiote_out_clear(MULTIPX_B2);

  adc_inited = true;

  return 0;
}

static uint8_t channel_map[ADC_CH_SIZE] = {
#if !defined(CHANNEL_16_23)
  2, 12, 1, 0, 3, 4, 6, 7,
  5, 11, 8, 9, 10, 13, 15, 14,
#else
  0, 1, 2, 3, 4, 5, 6, 7,
  8, 9, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23,
#endif
};

// it's a trigger.
int32_t adcx_channel_read(adc_channel_t channel)
{
  nrfx_err_t err;

  uint8_t bit = channel_map[channel];
#if defined(ENABLE_MULTPLEX3)
  if (bit > 15) {
    // bit -= 16;
    ((bit >> 0) & 0x01) ? nrfx_gpiote_out_set(MULTIPX_A3) : nrfx_gpiote_out_clear(MULTIPX_A3);
    ((bit >> 1) & 0x01) ? nrfx_gpiote_out_set(MULTIPX_B3) : nrfx_gpiote_out_clear(MULTIPX_B3);
    ((bit >> 2) & 0x01) ? nrfx_gpiote_out_set(MULTIPX_C3) : nrfx_gpiote_out_clear(MULTIPX_C3);
  } else
#endif
  if (bit >> 3 & 0x01) {
    ((bit >> 0) & 0x01) ? nrfx_gpiote_out_set(MULTIPX_A2) : nrfx_gpiote_out_clear(MULTIPX_A2);
    ((bit >> 1) & 0x01) ? nrfx_gpiote_out_set(MULTIPX_B2) : nrfx_gpiote_out_clear(MULTIPX_B2);
    ((bit >> 2) & 0x01) ? nrfx_gpiote_out_set(MULTIPX_C2) : nrfx_gpiote_out_clear(MULTIPX_C2);
  } else {
    ((bit >> 0) & 0x01) ? nrfx_gpiote_out_set(MULTIPX_A1) : nrfx_gpiote_out_clear(MULTIPX_A1);
    ((bit >> 1) & 0x01) ? nrfx_gpiote_out_set(MULTIPX_B1) : nrfx_gpiote_out_clear(MULTIPX_B1);
    ((bit >> 2) & 0x01) ? nrfx_gpiote_out_set(MULTIPX_C1) : nrfx_gpiote_out_clear(MULTIPX_C1);
  }
  // __wait // make sure gpio was config-ed.
   // 重新设置ADC缓冲区（确保缓冲区有效）
  err = nrfx_saadc_buffer_set(adc_value, SAADC_CHANNEL_SIZE);
  if (err != NRFX_SUCCESS) LOG_ERR("adc buffer set error: %08X", err);
   // 触发ADC转换（开始采样）
  err = nrfx_saadc_mode_trigger();
  if (err != NRFX_SUCCESS) LOG_ERR("adc mode trigger error: %08X", err);
  // nrfx_gpiote_out_clear(MULTIPX_A);
  // nrfx_gpiote_out_clear(MULTIPX_B);
  // nrfx_gpiote_out_clear(MULTIPX_C);
  // return adc_value[channel < 8 ? 0 : 1];
  return adc_value[bit / 8];
}

static volatile bool adc_task_is_running = false;
int32_t adcx_status(bool status)
{
  adc_task_is_running = status;
  return 0;
}

#endif

