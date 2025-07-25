#include "adc.h"
#include <nrfx_saadc.h>
 
int adc_init() {
	nrfx_saadc_init(7);

//channel1
	nrf_saadc_channel_config_t config1={ //初始化
		.gain = NRF_SAADC_GAIN1_4,
		.reference = NRF_SAADC_REFERENCE_INTERNAL,	//参考电压->内部电压
		.acq_time = NRF_SAADC_ACQTIME_10US,	//采样时间  
	};
	nrfx_saadc_channel_t channel1={
		.channel_config = config1,
		.pin_p =NRF_SAADC_INPUT_AIN6,
		.channel_index = 0,
	};
//channel2
	nrf_saadc_channel_config_t config2={ 
		.gain = NRF_SAADC_GAIN1_4,
		.reference = NRF_SAADC_REFERENCE_INTERNAL,	
		.acq_time = NRF_SAADC_ACQTIME_10US,	
	};
	nrfx_saadc_channel_t channel2={
		.channel_config = config2,
		.pin_p =NRF_SAADC_INPUT_AIN5,
		.channel_index = 1,
	};
//channel3
	nrf_saadc_channel_config_t config3={ 
		.gain = NRF_SAADC_GAIN1_4,
		.reference = NRF_SAADC_REFERENCE_INTERNAL,	
		.acq_time = NRF_SAADC_ACQTIME_10US,	 
	};
	nrfx_saadc_channel_t channel3={
		.channel_config = config3,
		.pin_p =NRF_SAADC_INPUT_AIN1,
		.channel_index = 2,//待定
	};
nrfx_saadc_channel_t channels[] = {
	channel1,
	channel2,
	channel3,
};
	return 0;
}

float *adc_read(int channel){
	nrf_saadc_value_t adc_buffer[3];
	nrfx_saadc_buffer_set(adc_buffer,3);
	nrfx_saadc_mode_trigger();
	while(nrfx_busy_check()){
		_WFI();
	} 
	nrfx_saadc_buffer_read(adc_buffer,3);
	int16_t	channel0 = adc_buffer[0];
	int16_t channel1 = adc_buffer[1];
	int16_t channel2 = adc_buffer[2];
	float * result = malloc(3*sizeof(float));
	result[0] = (float)channel0 / 4095.0*3.3;
	result[1] = (float)channel1 / 4095.0*3.3;
	result[2] = (float)channel2 / 4095.0*3.3;
	return result;
}