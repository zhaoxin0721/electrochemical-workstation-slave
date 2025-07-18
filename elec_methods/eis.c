#include "eis.h"
// #include "../drivers/impedance_ad594x.h"
// #include "../drivers/bat_impedance.h"
#include "../drivers/eis_impedance.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(elec_eis, LOG_LEVEL_INF);

static volatile bool working = false;
static elec_eis_config_t config = {
  .freq_begin = 10,
  .freq_end   = 10000,
  .sample_count = 100,
  .use_log    = true,
  .ac_volt    = 300.0f,
};

#define IMPAPPBUFF_SIZE 512
static uint32_t AppBuff[IMPAPPBUFF_SIZE];
static int  AD5940PlatformCfg(void);
static void AD5940ImpedanceStructInit(void);
static void elec_eis_loop();

int elec_eis_init()
{
  int err = AD5940PlatformCfg();
  if (err) { return -1; }
  AD5940ImpedanceStructInit();
  
  /* Initialize the application and configure EC sensor */
  AppIMPInit(AppBuff, IMPAPPBUFF_SIZE); 
  
  /* Control IMP measurment to start. Second parameter has no meaning with this command. */
  AppIMPCtrl(IMPCTRL_START, 0);    
  return 0;
}

int elec_eis_config(elec_eis_config_t config_)
{
  // copy
  config.freq_begin   = config_.freq_begin;
  config.freq_end     = config_.freq_end;
  config.sample_count = config_.sample_count;
  config.use_log      = config_.use_log;
  config.ac_volt      = config_.ac_volt;
  return 0;
}

int elec_eis_start()
{
  if (config.freq_begin < 1)    return -1;
  if (config.freq_end < 1)      return -2;
  if (config.sample_count < 1)  return -3;
  if (config.ac_volt < 1)       return -4;
  working = true;
  // new task:
  return elec_method_commit(&elec_eis_loop);
}

int elec_eis_stop()
{
  working = false;
  int err = elec_method_commit(NULL);
  LOG_INF("EIS stop, still working? %s", working ? "yes" : "no");
  return err;
}

bool elec_eis_is_working()
{
  return working;
}

// Implemention:

static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;

  AD5940_Initialize();
  /* Platform configuration */
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  
  
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);           /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);   /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter sleep mode. */
  return 0;
}

void AD5940ImpedanceStructInit(void)
{
  AppIMPCfg_Type *pImpedanceCfg;
  
  AppIMPGetCfg(&pImpedanceCfg);
  /* Step1: configure initialization sequence Info */
  pImpedanceCfg->SeqStartAddr = 0;
  pImpedanceCfg->MaxSeqLen = 512;
  
  pImpedanceCfg->RcalVal = 1000.0; // 200.0;
  pImpedanceCfg->SinFreq = 60000.0;
  pImpedanceCfg->FifoThresh = 6;
  pImpedanceCfg->ImpODR = 10;                /* 10 Hz output data rate rate */
  
  /* Configure EC Sensor Parameters */
  /*Sensor is connected to CH0 on EVAL-ADuCM355QSPZ */
  pImpedanceCfg->SensorCh0.LpTiaRf = LPTIARF_1M;         /* 1Mohm Rfilter, 4.7uF cap connected external on AIN4 */
  pImpedanceCfg->SensorCh0.LpTiaRl = LPTIARLOAD_10R;     /* 10ohm Rload */
  pImpedanceCfg->SensorCh0.LptiaRtiaSel = LPTIARTIA_200R;  /* Configure TIA gain resistor*/
  pImpedanceCfg->SensorCh0.Vzero = 1100;                  /* Set Vzero = 1100mV. Voltage on SE0 pin*/
  pImpedanceCfg->SensorCh0.SensorBias = config.ac_volt;              /* 0V bias voltage */
  
  /* Set switch matrix to connect to sensor in Ch0 for impedance measurement. */
  pImpedanceCfg->DswitchSel = SWD_CE0;
  pImpedanceCfg->PswitchSel = SWP_RE0;
  pImpedanceCfg->NswitchSel = SWN_SE0LOAD;
  pImpedanceCfg->TswitchSel = SWT_SE0LOAD;
  
/* The dummy sensor is as low as 5kOhm. We need to make sure RTIA is 
    small enough that HSTIA won't be saturated. */
  pImpedanceCfg->HstiaRtiaSel = HSTIARTIA_200;
  
  /* Configure the sweep function. */
  pImpedanceCfg->SweepCfg.SweepEn = bTRUE;
  pImpedanceCfg->SweepCfg.SweepStart = config.freq_begin;	/* Set start frequency*/
  pImpedanceCfg->SweepCfg.SweepStop = config.freq_end;	/* Set final frequency */
  pImpedanceCfg->SweepCfg.SweepPoints = config.sample_count;		/* Set number of points for sweep*/
  pImpedanceCfg->SweepCfg.SweepLog = config.use_log ? bTRUE : bFALSE;
  /* Configure Power Mode. Use HP mode if frequency is higher than 80kHz. */
  pImpedanceCfg->PwrMod = AFEPWR_LP;
  /* Configure filters if necessary */
  pImpedanceCfg->ADCSinc3Osr = ADCSINC3OSR_4;
  pImpedanceCfg->DftNum = DFTNUM_16384;
  pImpedanceCfg->DftSrc = DFTSRC_SINC3;

  pImpedanceCfg->DacVoltPP = config.ac_volt;
}

void elec_eis_loop()
{
  LOG_INF("Loop Start.");
  AD5940PlatformCfg();
  AD5940ImpedanceStructInit();
  uint32_t count;
  while(working)
  {
    if(AD5940_GetMCUIntFlag())
    {
      // LOG_INF("GPIO0 INT >>");
      AD5940_ClrMCUIntFlag();
      count = IMPAPPBUFF_SIZE;
      AppIMPISR(AppBuff, &count); 			/* Deal with it and provide a buffer to store data we got */
      // result
      fImpPol_Type *pImp = (fImpPol_Type*)AppBuff;
      float freq;
      AppIMPCtrl(IMPCTRL_GETFREQ, &freq);
      /*Process data*/
      for(int i=0;i<count;i++)
      {
        printk("Impedance[%f]: %f, %f\n", freq, pImp[i].Magnitude, pImp[i].Phase * 180 / MATH_PI);
      }
      // 选取中间点为标准值
      int selected_index = count / 2;
      // buffer => protocal
      uint32_t freq_ = (uint32_t)(freq * 100);
      int64_t real = isnan(pImp[selected_index].Magnitude) ? 0xFFFFFFFF : (int64_t)(pImp[selected_index].Magnitude * 100);
      int64_t imag = isnan(pImp[selected_index].Phase) ? 0xFFFFFFFF : (int64_t)(pImp[selected_index].Phase * 10000);
      uint8_t buffer[20];
      uint8_t index = 0;
      buffer[index++] = (uint8_t)(freq_ >> 24);
      buffer[index++] = (uint8_t)(freq_ >> 16);
      buffer[index++] = (uint8_t)(freq_ >> 8);
      buffer[index++] = (uint8_t)(freq_ & 0xFF);
      buffer[index++] = (uint8_t)(real >> 56);
      buffer[index++] = (uint8_t)(real >> 48);
      buffer[index++] = (uint8_t)(real >> 40);
      buffer[index++] = (uint8_t)(real >> 32);
      buffer[index++] = (uint8_t)(real >> 24);
      buffer[index++] = (uint8_t)(real >> 16);
      buffer[index++] = (uint8_t)(real >> 8);
      buffer[index++] = (uint8_t)(real & 0xFF);
      buffer[index++] = (uint8_t)(imag >> 56);
      buffer[index++] = (uint8_t)(imag >> 48);
      buffer[index++] = (uint8_t)(imag >> 40);
      buffer[index++] = (uint8_t)(imag >> 32);
      buffer[index++] = (uint8_t)(imag >> 24);
      buffer[index++] = (uint8_t)(imag >> 16);
      buffer[index++] = (uint8_t)(imag >> 8);
      buffer[index++] = (uint8_t)(imag & 0xFF);

      send_fac_stream_directly(buffer, 20);

      // break
      if (freq >= config.freq_end) {
        break;
      }
    }
    k_msleep(10);
  }
  LOG_INF("Loop End.");
  AppIMPCtrl(IMPCTRL_STOPNOW, 0);
  k_msleep(10);
  uint8_t stop_signal[2];
  stop_signal[0] = 0xFF;
  stop_signal[1] = 0xFF;
  send_fac_stream_directly(stop_signal, 2);
  elec_eis_stop();
}

