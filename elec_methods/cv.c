#define DEBUG

#include "cv.h"
#include "../drivers/freistat_ramp.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(elec_cv, LOG_LEVEL_INF);

void elec_cv_loop();
static volatile bool working = false;
static elec_cv_config_t config = {
  .vertex_begin   = -1000.0f,           /* -1V */
  .vertex_1    = +1000.0f,           /* +1V */
  .vertex_2  = 1100.0f,            /* 1.3V */
  .step_value   = 1100.0f,             /* 1.3V */
  .sample_delay = 7.0f,               /* 7ms. Time between update DAC and ADC sample. Unit is ms. */
  .scan_rate  = 1000,                /* Total steps. Equals to ADC sample number */
  .cycle_number = 0,
  .rtia = LPTIARTIA_8K,
};
static uint32_t index;

int elec_cv_init()
{
  return 0;
}

int elec_cv_config(elec_cv_config_t config_)
{
  config.vertex_begin   = config_.vertex_begin;
  config.vertex_1       = config_.vertex_1;
  config.vertex_2       = config_.vertex_2 ;
  config.step_value     = config_.step_value;
  config.scan_rate      = config_.scan_rate;
  config.sample_delay   = config_.sample_delay;
  config.cycle_number   = config_.cycle_number;
  config.rtia           = config_.rtia;
  return 0;
}

int elec_cv_start()
{
  index = 0;
  working = true;
  return elec_method_commit(&elec_cv_loop);
}

int elec_cv_stop()
{
  index = 0;
  working = false;
  int err = elec_method_commit(NULL);
  LOG_INF("CV stop: still working? %s", working ? "yes" : "no");
  return err;
}

bool elec_cv_is_working()
{
  return working;
}

// Implementation:
#define RAMP_APPBUFF_SIZE 1024

uint32_t RampAppBuff[RAMP_APPBUFF_SIZE]; //buffer to fetch AD5940 samples
float RampLFOSCFreq;                /* Measured LFOSC frequency */

static volatile int current_cycle_index = 0;

AD5940Err AppRAMPCycleDone(uint16_t CurrentCycleIndex)
{
  LOG_INF("New Cycle: %d.", CurrentCycleIndex);
  current_cycle_index = CurrentCycleIndex;
  return 0;
}

/**
 * @brief An example to deal with data read back from AD5940. Here we just print data to UART
 * @note UART has limited speed, it has impact when sample rate is fast. Try to print some of the data not all of them.
 * @param pData: the buffer stored data for this application. The data from FIFO has been pre-processed.
 * @param DataCount: The available data count in buffer pData.
 * @return return 0.
*/
static int32_t RampShowResult(float *pData, uint32_t DataCount)
{
#define RAMP_DATA_BUF_MOD  2
#define RAMP_DATA_BUF_SIZE (RAMP_DATA_BUF_MOD * 10)
  /* Print data*/
  int err = 0;
  uint8_t data[RAMP_DATA_BUF_SIZE]; // 3*(2+4+4)=30
  for(int i=0; i<DataCount * 2; )
  {
    if (index % 10 == 0) { // filter.
      LOG_INF("[%d/%d], %.3f mV => %.3f uA", i, DataCount, pData[i], pData[i + 1]);
    }

    //i += 10;  /* Print though UART consumes too much time. */
    uint16_t id = index;
    int32_t uV = (int32_t)(pData[i++] * 1000); // to uV
    int32_t nA = (int32_t)(pData[i++] * 1000); // to nA
    int offset = index % RAMP_DATA_BUF_MOD;
    offset *= 10;
    data[offset++] = (uint8_t)((current_cycle_index >> 8) & 0xFF);
    data[offset++] = (uint8_t)(current_cycle_index & 0xFF);
    data[offset++] = (uint8_t)((nA >> 24) & 0xFF);
    data[offset++] = (uint8_t)((nA >> 16) & 0xFF);
    data[offset++] = (uint8_t)((nA >> 8) & 0xFF);
    data[offset++] = (uint8_t)((nA) & 0xFF);
    data[offset++] = (uint8_t)((uV >> 24) & 0xFF);
    data[offset++] = (uint8_t)((uV >> 16) & 0xFF);
    data[offset++] = (uint8_t)((uV >> 8) & 0xFF);
    data[offset++] = (uint8_t)((uV) & 0xFF);
    if (index % RAMP_DATA_BUF_MOD == (RAMP_DATA_BUF_MOD - 1)) {
      k_msleep(5);
      err = send_fac_stream_directly(data, RAMP_DATA_BUF_SIZE);
      if (err) {
        // retry--
        k_msleep(2);
        err = send_fac_stream_directly(data, RAMP_DATA_BUF_SIZE);
        if (err) {
          LOG_ERR("Ble Face Error: %08X", err);
        }
      }
    }
    index++;
  }
  return 0;
}

/**
 * @brief The general configuration to AD5940 like FIFO/Sequencer/Clock. 
 * @note This function will firstly reset AD5940 using reset pin.
 * @return return 0.
*/
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize(); /* Call this right after AFE reset */
  /* Platform configuration */
  /* Step1. Configure clock */
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  /* Configure FIFO and Sequencer */
  fifo_cfg.FIFOEn = bTRUE; /* We will enable FIFO after all parameters configured */
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_2KB; /* 2kB for FIFO, The reset 4kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3; /* */
  fifo_cfg.FIFOThresh = 4;          /*  Don't care, set it by application paramter */
  AD5940_FIFOCfg(&fifo_cfg);
  seq_cfg.SeqMemSize = SEQMEMSIZE_4KB; /* 4kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bTRUE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT0 | AFEINTSRC_CUSTOMINT1 | AFEINTSRC_GPT1INT_TRYBRK | AFEINTSRC_DATAFIFOOF, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Configure GPIO */
  gpio_cfg.FuncSet = GP0_INT | GP1_GPIO | GP2_SYNC; /* GPIO1 indicates AFE is in sleep state. GPIO2 indicates ADC is sampling. */
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0 | AGPIO_Pin1 | AGPIO_Pin2;
  gpio_cfg.OutVal = AGPIO_Pin1; //set high to turn off LED
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  /* Measure LFOSC frequency */
  /**@note Calibrate LFOSC using system clock. The system clock accuracy decides measurement accuracy. Use XTAL to get better result. */
  LfoscMeasure.CalDuration = 1000.0;        /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;              /* Put sequence commands from start address of SRAM */
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, &RampLFOSCFreq);
#ifdef DEBUG
  printk("Measured LFOSC Freq (used for sequencer timing) - rounded: %d Hz\n", (int)(RampLFOSCFreq + 0.5));
#endif
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); /*  */
  return 0;
}

/**
 * @brief The interface for user to change application parameters. All parameters belong to the AppRAMPCfg (see RampTest.c file)
 * @return return 0.
*/
void AD5940RampStructInit(void)
{
  AppRAMPCfg_Type *pRampCfg;

  AppRAMPGetCfg(&pRampCfg);
  /* Step1: configure general parameters */
  pRampCfg->SeqStartAddr = 0x10;      /* leave 16 commands for LFOSC calibration.  */
  pRampCfg->MaxSeqLen = 1024 - 0x10;  /* 4kB/4 = 1024  */
  pRampCfg->RcalVal = 1000.0;         /* 1kOhm RCAL */
  pRampCfg->ADCRefVolt = 1820.0f;     /* The real ADC reference voltage. Measure it from capacitor C3 (AD5941 FeatherWing Rev1 Board) with DMM. */
  pRampCfg->FifoThresh = 100;         /* Maximum value is 2kB/4-1 = 512-1. Set it to higher value to save power. */
  pRampCfg->SysClkFreq = 16000000.0f; /* System clock is 16MHz by default */
  pRampCfg->LFOSCClkFreq = RampLFOSCFreq; /* LFOSC frequency */
  /* Configure ramp signal parameters */
  pRampCfg->RampStartVolt = config.vertex_begin;  /* in mV*/
  pRampCfg->RampPeakVolt1 = config.vertex_1; /* If FIX_WE_POT defined, make sure (|RampPeakVolt1 - RampPeakVolt2| + 35mV) <= (VzeroHighLevel-VzeroLowLevel) */
  pRampCfg->RampPeakVolt2 = config.vertex_2; //-100.0f;
  pRampCfg->VzeroLimitHigh = 2400;   /* 2.2V */
  pRampCfg->VzeroLimitLow = 200;     /* 0.4V */
  pRampCfg->Estep = config.step_value; // <100mV
  pRampCfg->ScanRate = config.scan_rate;
  pRampCfg->CycleNumber = config.cycle_number;

  pRampCfg->ADCSinc3Osr = ADCSINC3OSR_4;
  pRampCfg->ADCSinc2Osr = ADCSINC2OSR_667;

  pRampCfg->SampleDelay = config.sample_delay;           /* 7ms. Time between update DAC and ADC sample. Unit is ms. SampleDelay > 1.0ms is acceptable.*/
  pRampCfg->LpAmpPwrMod = LPAMPPWR_NORM;  /* restrict to max. +/- 750 uA cell current*/
  pRampCfg->LPTIARtiaSel = config.rtia; //  LPTIARTIA_32K; /* Maximum current decides RTIA value: Imax = 0.9V / RTIA */
  pRampCfg->LPTIARloadSel = LPTIARLOAD_SHORT;
  pRampCfg->AdcPgaGain = ADCPGA_1P5;

  pRampCfg->bRampOneDir = bFALSE; //activate LSV instead of CV

#ifdef DEBUG
  printk("Ramp Parameters:\n");
  if (pRampCfg->bRampOneDir)
  {
    printk("Setup for Linear Sweep Voltammetry!\n");
  }
  else
  {
    printk("Setup for Cyclic Voltammetry!\n");
  }
  printk("- Vstart = %d mV\n", (int)pRampCfg->RampStartVolt);
  printk("- V1 = %d mV\n", (int)pRampCfg->RampPeakVolt1);
  printk("- V2 = %d mV\n", (int)pRampCfg->RampPeakVolt2);
  printk("- Estep = %d mV\n", (int)pRampCfg->Estep);
  printk("- Scan Rate = %d mV/s\n", (int)pRampCfg->ScanRate);
  printk("- Number of Cycles = %d\n", (int)pRampCfg->CycleNumber);
  int Sinc3OSR[] = {2, 4, 5};
  int Sinc2OSR[] = {22, 44, 89, 178, 267, 533, 640, 667, 800, 889, 1067, 1333};
  int SampleRate = (int)(800000.0 / Sinc3OSR[pRampCfg->ADCSinc3Osr] / Sinc2OSR[pRampCfg->ADCSinc2Osr] + 0.5);
  printk("- Sampling Freq (rounded) = %d Sps\n", (int)SampleRate);
  //check timing parameters
  //necessary time after each step: DAC code write (~3.1ms for StepsPerBlock = 1) + get remaining FIFO data from last step and process (~3.2ms / 50 samples in FIFO) + print output (~0.5ms)
  //available time after each step: SampleDelay + min(Estep/ScanRate in ms - SampleDelay, FIFOThresh / fSample)
  float a = pRampCfg->Estep / pRampCfg->ScanRate * 1000.0 - pRampCfg->SampleDelay;
  float b = pRampCfg->FifoThresh / ((float)SampleRate) * 1000;
  //float c = min(a,b);
  // if ((3.1 + 3.2 * pRampCfg->FifoThresh / 50 + 0.5) >=
  //     (pRampCfg->SampleDelay + min(a, b)))
  // {
  //   printk("WARNING: Timing critical - Increase Estep, decrease ScanRate or adapt FIFO threshold\n");
  // }
#endif
}

void elec_cv_loop()
{
  //configure AFE
  AD5940PlatformCfg();
  //init application with pre-defined parameters
  AD5940RampStructInit();

  uint32_t temp;
  AppRAMPCfg_Type *pRampCfg;

  AppRAMPInit(RampAppBuff, RAMP_APPBUFF_SIZE); /* Initialize RAMP application. Provide a buffer, which is used to store sequencer commands */

  //send ramp data to python application
#ifdef PLOT_DATA
  AppRAMPGetCfg(&pRampCfg);
  int Sinc3OSR[] = {2, 4, 5};
  int Sinc2OSR[] = {22, 44, 89, 178, 267, 533, 640, 667, 800, 889, 1067, 1333};
  //pack voltage and current in a byte array
  uint8_t byteData[30];
  byteData[0] = ((int16_t)pRampCfg->RampStartVolt & 0xFF00) >> 8;
  byteData[1] = (int16_t)pRampCfg->RampStartVolt & 0x00FF;
  byteData[2] = ((int16_t)pRampCfg->RampPeakVolt1 & 0xFF00) >> 8;
  byteData[3] = (int16_t)pRampCfg->RampPeakVolt1 & 0x00FF;
  byteData[4] = ((int16_t)pRampCfg->RampPeakVolt2 & 0xFF00) >> 8;
  byteData[5] = (int16_t)pRampCfg->RampPeakVolt2 & 0x00FF;
  byteData[6] = ((int16_t)pRampCfg->Estep & 0xFF00) >> 8;
  byteData[7] = (int16_t)pRampCfg->Estep & 0x00FF;
  byteData[8] = ((int16_t)pRampCfg->ScanRate & 0xFF00) >> 8;
  byteData[9] = (int16_t)pRampCfg->ScanRate & 0x00FF;
  byteData[10] = ((int16_t)pRampCfg->CycleNumber & 0xFF00) >> 8;
  byteData[11] = (int16_t)pRampCfg->CycleNumber & 0x00FF;
  byteData[12] = ((int16_t)Sinc3OSR[pRampCfg->ADCSinc3Osr] & 0xFF00) >> 8;
  byteData[13] = (int16_t)Sinc3OSR[pRampCfg->ADCSinc3Osr] & 0x00FF;
  byteData[14] = ((int16_t)Sinc2OSR[pRampCfg->ADCSinc2Osr] & 0xFF00) >> 8;
  byteData[15] = (int16_t)Sinc2OSR[pRampCfg->ADCSinc2Osr] & 0x00FF;
  byteData[16] = ((int16_t)pRampCfg->StepNumber & 0xFF00) >> 8;
  byteData[17] = (int16_t)pRampCfg->StepNumber & 0x00FF;
  //write the bytes to serial port
  Serial.write(&byteData[0], 18);
  //delay to allow python to set everything up
  delay(3000);
#endif

#ifdef DEBUG
  printk("All initialized.\n");
  printk("---start of voltammetry ---\n");
#endif
  AppRAMPCtrl(APPCTRL_START, 0); /* Control RAMP measurement to start. Second parameter has no meaning with this command. */

  while (working && !pRampCfg->bTestFinished)
  {
    AppRAMPGetCfg(&pRampCfg);
    if (AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();
      temp = RAMP_APPBUFF_SIZE;
      AppRAMPISR(RampAppBuff, &temp); //temp now holds the number of calculated means (>0, if at least one step was finished within the current data set from FIFO)
      //print data in case at least one step was finished
      if (temp > 0)
      {
        RampShowResult((float *)RampAppBuff, temp);
      }
    }
  }

  //end of test
#ifdef DEBUG
  printk("---end of ramp test---\n");
#endif

  //after test finished, reset flag to be able to start new test
  pRampCfg->bTestFinished = bFALSE;
  AppRAMPCtrl(APPCTRL_STOPSYNC, 0);
  AppRAMPCtrl(APPCTRL_STOPNOW, 0);
  AppRAMPCtrl(APPCTRL_SHUTDOWN, 0);

  k_msleep(10);
  uint8_t stop_signal[2];
  stop_signal[0] = 0xFF;
  stop_signal[1] = 0xFF;
  send_fac_stream_directly(stop_signal, 2);
  elec_cv_stop();
}
