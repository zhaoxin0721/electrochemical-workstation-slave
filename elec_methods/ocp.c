
#include "ocp.h"
#include "../drivers/ad594x.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(elec_ocp, LOG_LEVEL_INF);

static volatile bool working = false;
// static volatile send_fac_stream_t send_fac_stream_fun = NULL;
static opc_config_t config;
static volatile uint16_t config_sample_coef = 150;

#define ADCPGA_GAIN_SEL   ADCPGA_1 // ADCPGA_1P5
static void AD5940_PGA_Calibration(void){
  AD5940Err err;
  ADCPGACal_Type pgacal;
  pgacal.AdcClkFreq = 16e6;
  pgacal.ADCSinc2Osr = ADCSINC2OSR_178;
  pgacal.ADCSinc3Osr = ADCSINC3OSR_4;
  pgacal.SysClkFreq = 16e6;
  pgacal.TimeOut10us = 1000;
  pgacal.VRef1p11 = 1.11f;
  pgacal.VRef1p82 = 1.82f;
  pgacal.PGACalType = PGACALTYPE_OFFSETGAIN;
  pgacal.ADCPga = ADCPGA_GAIN_SEL;
  err = AD5940_ADCPGACal(&pgacal);
  if(err != AD5940ERR_OK){
    LOG_INF("AD5940 PGA calibration failed.");
  }
}

void elec_ocp_init(void)
{
  AFERefCfg_Type aferef_cfg;
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;
  SWMatrixCfg_Type sw_matrix;
  
  /* Use hardware reset */
  AD5940_HWReset();

  /* Firstly call this function after reset to initialize AFE registers. */
  AD5940_Initialize();
  
  AD5940_PGA_Calibration();
  /* Configure AFE power mode and bandwidth */
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);

  /* Initialize ADC basic function */
  AD5940_AFECtrlS(AFECTRL_DACREFPWR|AFECTRL_HSDACPWR, bTRUE); //We are going to measure DAC 1.82V reference.
  adc_base.ADCMuxP = ADCMUXP_AIN1; // ADCMUXP_VRE0; // ADCMUXP_VREF1P8DAC;
  adc_base.ADCMuxN = ADCMUXN_VSET1P1; // ⚠️！！此处若更换为 AIN1 则会测不准 // ADCMUXN_VSET1P1;
  // adc_base.ADCMuxP = ADCMUXP_VREF1P8DAC;
  // adc_base.ADCMuxN = ADCMUXN_VSET1P1;
  adc_base.ADCPga = ADCPGA_GAIN_SEL;
  AD5940_ADCBaseCfgS(&adc_base);
  
  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH */
  adc_filter.ADCSinc3Osr = ADCSINC3OSR_4;
  adc_filter.ADCSinc2Osr = ADCSINC2OSR_1333;
  adc_filter.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  adc_filter.BpNotch = bTRUE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. */
  adc_filter.BpSinc3 = bFALSE;                /* We use SINC3 filter. */   
  adc_filter.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */ 
  AD5940_ADCFilterCfgS(&adc_filter);
  
  // AD5940_ADCMuxCfgS(ADCMUXP_AIN0, ADCMUXN_VSET1P1);   /* Optionally, change ADC MUX */

  // sw_matrix.Dswitch = SWD_AIN1;
  // sw_matrix.Pswitch = SWP_RE0;
  // sw_matrix.Nswitch = SWN_AIN2;
  // sw_matrix.Tswitch = SWT_AIN0;
  // sw_matrix.Dswitch = SWD_OPEN;
  // sw_matrix.Pswitch = SWP_RE1|SWP_DE0;
  // sw_matrix.Nswitch = SWN_AIN2|SWN_SE0;
  // sw_matrix.Tswitch = SWT_AIN2|SWT_AFE3LOAD;
  // AD5940_SWMatrixCfgS(&sw_matrix);

  AD5940_AFECtrlS(AFECTRL_HPREFPWR, bTRUE); /* Enable reference. It's automatically turned off during hibernate */

  /* Enable all interrupt at Interrupt Controller 1. So we can check the interrupt flag */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); 

  //AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bTRUE);
  //AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);
  AD5940_ADCPowerCtrlS(bTRUE);
  AD5940_ADCConvtCtrlS(bTRUE);
}

void elec_ocp_loop() {
  LOG_INF("ElectrochemicalMethod: OCP working...");
  uint8_t buffer[4];
  buffer[0] = 0; buffer[1] = 0; buffer[2] = 0; buffer[3] = 0;
  // uint8_t index = 0;
  // delay
  if (config.delay > 0) {
    uint16_t each_delay_ms = 1000 / config.sample_frequency;
    int16_t rest_ms = config.delay;
    while(rest_ms >= 0) {
      send_fac_stream(buffer, 4); // real-time notify.
      k_msleep(each_delay_ms);
      rest_ms -= each_delay_ms;
    }
  }
  while(working)
  {
    uint32_t rd;
    if(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_SINC2RDY))  
    {
      static uint32_t count;
      AD5940_INTCClrFlag(AFEINTSRC_SINC2RDY);
      rd = AD5940_ReadAfeResult(AFERESULT_SINC2);
      count ++;
      /* ADC Sample rate is 800kSPS. SINC3 OSR is 4, SINC2 OSR is 1333. So the final output data rate is 800kSPS/4/1333 = 150.0375Hz */
      // if(count == 150) /* Print data @1Hz */
      // {
      //   count = 0;
      //   // float diff_volt = AD5940_ADCCode2Volt(rd, ADCPGA_GAIN_SEL, 1.82);
      //   // LOG_INF("ADC Code:%d,\tdiff-volt: %d,\tvolt:%d mV",rd, v, v+1110);
      //   float diff_volt = AD5940_ADCCode2Volt(rd, ADCPGA_GAIN_SEL, 1.82);
      //   int v = diff_volt * 1000;
      //   LOG_INF("ADC Code:%d,\tdiff-volt: %d,\tvolt:%d mV",rd, v, v);
      // }
      if(count == config_sample_coef) /* Print data @10Hz */
      {
        count = 0;
        float volt = AD5940_ADCCode2Volt(rd, ADCPGA_GAIN_SEL, 1.82);// + 1.11f;
        int32_t uVolt = volt * 10e6;
        LOG_INF("ADC Code:%d,\t volt:%fV", rd, volt);
        uint32_t data = (uint32_t)(uVolt + (2 << 31)); // remove negative sign.
        // save to buffer
        buffer[0] = data >> 24 & 0xFF;
        buffer[1] = data >> 16 & 0xFF;
        buffer[2] = data >> 8 & 0xFF;
        buffer[3] = data & 0xFF;
        // LOG_INF("Data: %02X %02X %02X %02X", buffer[0], buffer[1], buffer[2], buffer[3]);
        send_fac_stream(buffer, 4); // real-time notify.
      }
    }
  }
  AD5940_ADCPowerCtrlS(bFALSE);
  AD5940_ADCConvtCtrlS(bFALSE);
}

void elec_ocp_loop_test() {
  uint8_t buffer[19];
  uint8_t index = 0;
  while(working) {
    buffer[index++] = index;
    if (index >= 19)
    {
      send_fac_stream(buffer, index);
      index = 0;
    }
    k_msleep(10);
  }
}

// int elec_ocp_start(send_fac_stream_t send_fac_stream) {
int elec_ocp_start() {
  LOG_INF("ElectrochemicalMethod: OCP start.");
  working = true;
  // send_fac_stream_fun = send_fac_stream;
  // new task:
  return elec_method_commit(&elec_ocp_loop);
}

int elec_ocp_stop()
{
  LOG_INF("ElectrochemicalMethod: OCP stop.");
  working = false;
  int err = elec_method_commit(NULL);

  LOG_INF("OCP stop, still working? %s", working ? "yes" : "no");
  return err;
}

bool elec_ocp_is_working()
{
  return working;
}

int elec_ocp_config(opc_config_t config_)
{
  // copy values.
  config.sample_frequency = config_.sample_frequency;
  config.delay = config_.delay;
  // range validator.
  if (config.sample_frequency == 0) { config.sample_frequency = 1; }
  if (config.sample_frequency > 50) { config.sample_frequency = 50; }
  config_sample_coef = 150 / config.sample_frequency;
  if (config.delay > 10000) { config.delay = 10000; }
  LOG_INF("OCP Config Update: sample_frequency=%d, delay=%d", config.sample_frequency, config.delay);
  return 0;
}