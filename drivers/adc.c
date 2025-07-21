#include "adc.h"
#ifndef ENABLE_NRF_ADC

#include <stdint.h>
#include <drivers/adc.h>
#include <nrfx_gpiote.h>

//ADC
#define ADC_CHANNEL       1 //定义ADC通道号为1

#define ADC_DEVICE_NAME		DT_NODELABEL(adc)
#define ADC_RESOLUTION		14
#define ADC_GAIN			    ADC_GAIN_1 
#define ADC_REFERENCE		  ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 20)
#define BUFFER_SIZE			  6

#define BAD_ANALOG_READ   -123

static bool _IsInitialized = false;
static uint8_t _LastChannel = 250;
static int16_t m_sample_buffer[BUFFER_SIZE];


// the channel configuration with channel not yet filled in
static struct adc_channel_cfg m_1st_channel_cfg = {
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = 0, // gets set during init
	.differential	    = 0,
#if CONFIG_ADC_CONFIGURABLE_INPUTS
	.input_positive   = 0, // gets set during init
#endif
};

const struct device *adc_dev = DEVICE_DT_GET(ADC_DEVICE_NAME);
// initialize the adc channel
static const struct device* init_adc()
{
	int ret;
  int channel = ADC_CHANNEL;
	if(_LastChannel != channel)
	{
		_IsInitialized = false;
		_LastChannel = channel;
	}

	if ( adc_dev != NULL && !_IsInitialized)
	{
		// strangely channel_id gets the channel id and input_positive gets id+1
		m_1st_channel_cfg.channel_id = channel;
#if CONFIG_ADC_CONFIGURABLE_INPUTS
        m_1st_channel_cfg.input_positive = channel+1,
#endif
		ret = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
		if(ret != 0)
		{
			//LOG_INF("Setting up of the first channel failed with code %d", ret);
			adc_dev = NULL;
		}
		else
		{
			_IsInitialized = true;	// we don't have any other analog users
		}
	}
	
	memset(m_sample_buffer, 0, sizeof(m_sample_buffer));
	return adc_dev;
}

const struct adc_sequence sequence = {
  .options     = NULL,				// extra samples and callback
  .channels    = BIT(ADC_CHANNEL),		// bit mask of channels to read
  .buffer      = m_sample_buffer,		// where to put samples read
  .buffer_size = sizeof(m_sample_buffer),
  .resolution  = ADC_RESOLUTION,		// desired resolution
  .oversampling = 0,					// don't oversample
  .calibrate = 0						// don't calibrate
};

// ------------------------------------------------
// read one channel of adc
// ------------------------------------------------
static int16_t read_adc_data()
{
	int ret;
	int16_t sample_value = BAD_ANALOG_READ;
	if (adc_dev)
	{
		ret = adc_read(adc_dev, &sequence);
		if(ret == 0)
		{
			sample_value = m_sample_buffer[0];
		}
	}

	return sample_value;
}

// ------------------------------------------------
// high level read adc channel and convert to float voltage
// ------------------------------------------------
float analog_read(float scale)
{

	int16_t sv = read_adc_data();
	if(sv == BAD_ANALOG_READ)
	{
		return sv;
	}
  // return (sv * scale) / ADC_RESOLUTION;
	// Convert the result to voltage
	// Result = [V(p) - V(n)] * GAIN/REFERENCE / 2^(RESOLUTION)
																				  
	int multip = 256;
	// find 2**adc_resolution
	switch(ADC_RESOLUTION)
	{
		default :
		case 8 :
			multip = 256;
			break;
		case 10 :
			multip = 1024;
			break;
		case 12 : // target
			multip = 4096;
			break;
		case 14 :
			multip = 16384;
			break;
	}
	float fout = (sv * scale / multip); // 需要实际电路二次转换后使用，此处为读取值
	return fout;
}

// adc.h

#define MULTIPX_A     NRF_GPIO_PIN_MAP(0, 02)   //端口 0 的第 2 号引脚（可用作 ADC0）
#define MULTIPX_B     NRF_GPIO_PIN_MAP(0, 03)	//端口 0 的第 3 号引脚（可用作 ADC0）
#define MULTIPX_C     NRF_GPIO_PIN_MAP(0, 04)	//端口 0 的第 4 号引脚（可用作 ADC0）
#define SAADC_AIN_A   6
#define SAADC_AIN_B   5

int adc_init()
{
  init_adc(); // channel AIN7.
  nrfx_gpiote_output_config_t a = NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
  nrfx_gpiote_output_configure(MULTIPX_A, &a, NULL);
  nrfx_gpiote_output_config_t b = NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
  nrfx_gpiote_output_configure(MULTIPX_B, &b, NULL);
  nrfx_gpiote_output_config_t c = NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
  nrfx_gpiote_output_configure(MULTIPX_C, &c, NULL);
  nrfx_gpiote_input_config_t in1 = NRFX_GPIOTE_DEFAULT_INPUT_CONFIG;
  nrfx_gpiote_input_configure(NRF_GPIO_PIN_MAP(0, 30), &in1, NULL, NULL);
  nrfx_gpiote_input_config_t in2 = NRFX_GPIOTE_DEFAULT_INPUT_CONFIG;
  nrfx_gpiote_input_configure(NRF_GPIO_PIN_MAP(0, 29), &in2, NULL, NULL);
  return 0;
}

#define __wait                for(int t = 0; t < 1000; t++) { __NOP(); } // k_usleep(2);

float adc_channel_read(adc_channel_t channel)
{
  // uint8_t bit;
  // uint8_t ain = 7;
  // if (channel < 8) {
  //   bit = channel & 0xFF;
  //   ain = SAADC_AIN_A;
  // } else {
  //   bit = (channel - 8) & 0xFF;
  //   ain = SAADC_AIN_B;
  // }
  uint8_t bit = channel;
  ((bit >> 0) & 0x01) ? nrfx_gpiote_out_set(MULTIPX_A) : nrfx_gpiote_out_clear(MULTIPX_A);
  ((bit >> 1) & 0x01) ? nrfx_gpiote_out_set(MULTIPX_B) : nrfx_gpiote_out_clear(MULTIPX_B);
  ((bit >> 2) & 0x01) ? nrfx_gpiote_out_set(MULTIPX_C) : nrfx_gpiote_out_clear(MULTIPX_C);
  __wait
  __wait
  float volt = analog_read(3700) - 1700;
  nrfx_gpiote_out_clear(MULTIPX_A);
  nrfx_gpiote_out_clear(MULTIPX_B);
  nrfx_gpiote_out_clear(MULTIPX_C);
  return volt;
}

#endif
