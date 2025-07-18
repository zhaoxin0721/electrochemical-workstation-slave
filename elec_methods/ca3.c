#include "ca.h"
#include "../drivers/chrono_amp_simple.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(elec_ca, LOG_LEVEL_INF);

void elec_ca_loop();
volatile bool  ca_is_working = false;
static elec_ca_config_t config = {
  .amplitude = 500.0f,
  .ms = 1000,
};


int elec_ca_init()
{
  // do nothing.
  return 0;
}

int elec_ca_config(elec_ca_config_t config_)
{
  // copy
  config.amplitude = config_.amplitude;
  config.ms = config_.ms;
  return 0;
}

int elec_ca_start()
{
  // config valid?
  ca_is_working = true;
  // new task:
  return elec_method_commit(&elec_ca_loop);
}

int elec_ca_stop()
{
  ca_is_working = false;
  int err = elec_method_commit(NULL);
  LOG_INF("CA stop: still working? %s", ca_is_working ? "yes" : "no");
  return err;
}

bool elec_ca_is_working()
{
  return ca_is_working;
}

// Implemention:

#define APPBUFF_SIZE 600
#define n 3
uint32_t AppBuff[n][APPBUFF_SIZE];

float LFOSCFreq;
uint32_t IntCount = 0;

static uint32_t pindex;//position in dataset

/* It's your choice here what to do with the data. Here is just an example to print to UART */
int32_t AMPShowResult(bool is_pluse, float *pData, uint32_t DataCount)
{
  #define CA_DATA_NUM 1  // 30*(2+4+1+1)=240
  #define CA_DATA_LEN (10 * CA_DATA_NUM) // 30*(2+4+1+1)=240
  uint8_t data[CA_DATA_LEN];
  for(int i=0;i<DataCount;i++)
  {
    printk("index: %d, Current: %fuA\n", pindex, pData[i]);
    if (is_pluse) {
      break;
    }
    float value = pData[i];
    uint16_t id = pindex;
    int32_t val = (int32_t)(value * 1000); // to nA or mV
    int offset = pindex % CA_DATA_NUM;
    offset *= 10;
    data[offset++] = (uint8_t)((id >> 24) & 0xFF);
    data[offset++] = (uint8_t)((id >> 16) & 0xFF);
    data[offset++] = (uint8_t)((id >> 8) & 0xFF);
    data[offset++] = (uint8_t)(id & 0xFF);
    data[offset++] = (uint8_t)((val >> 24) & 0xFF);
    data[offset++] = (uint8_t)((val >> 16) & 0xFF);
    data[offset++] = (uint8_t)((val >> 8) & 0xFF);
    data[offset++] = (uint8_t)((val) & 0xFF);
    data[offset++] = 0x00; // V / uA
    data[offset++] = 0x00; // ca method: I V_SE V_RE
    pindex++;
    if (pindex % CA_DATA_NUM == 0) {
      k_msleep(5);
      int err = send_fac_stream_directly(data, CA_DATA_LEN);
      if (err) {
        // retry--
        k_msleep(2);
        err = send_fac_stream_directly(data, CA_DATA_LEN);
        if (err) {
          LOG_ERR("Ble Face Error: %08X", err);
        }
      }
    }
  }
  return 0;
}

/* Initialize AD5940 basic blocks like clock */
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;
	LFOSCMeasure_Type LfoscMeasure;

/* Use hardware reset */
  AD5940_HWReset();

  /* Platform configuration */
  AD5940_Initialize();
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
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;//AppAMPCfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);                             /* Disable to reset FIFO. */
	fifo_cfg.FIFOEn = bTRUE;  
  AD5940_FIFOCfg(&fifo_cfg);                             /* Enable FIFO here */
  
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);           /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH|AFEINTSRC_ENDSEQ, bTRUE);   /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP6_SYNC|GP5_SYNC|GP4_SYNC|GP2_TRIG|GP1_SYNC|GP0_INT;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
	
	AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Enable AFE to enter sleep mode. */
  /* Measure LFOSC frequency */
  LfoscMeasure.CalDuration = 1000.0;  /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);
  printk("Freq:%f\n", LFOSCFreq); 
	
  return 0;
}

/* !!Change the application parameters here if you want to change it to none-default value */
void AD5940AMPStructInit(void)
{
  AppCHRONOAMPCfg_Type *pAMPCfg; 
  AppCHRONOAMPGetCfg(&pAMPCfg);
  /* Configure general parameters */
	pAMPCfg->WuptClkFreq = LFOSCFreq;					/* Use measured 32kHz clock freq for accurate wake up timer */
  pAMPCfg->SeqStartAddr = 0;
  pAMPCfg->MaxSeqLen = 512; 								/* @todo add checker in function */
  pAMPCfg->RcalVal = 1000.0;
  pAMPCfg->NumOfData = -1;      						/* Never stop until you stop it manually by AppAMPCtrl() function */
	
	pAMPCfg->AmpODR = 1;
	pAMPCfg->FifoThresh = 1; // 5
	pAMPCfg->ADCRefVolt = 1.82;							/* Measure voltage on VREF_1V8 pin and add here */
	
	pAMPCfg->ExtRtia = bFALSE;			/* Set to true if using external Rtia */
	pAMPCfg->ExtRtiaVal = 10000000; /* Enter external Rtia value here is using one */
	pAMPCfg->LptiaRtiaSel = LPTIARTIA_1K;		/* Select TIA gain resistor. */
	
	pAMPCfg->SensorBias = config.amplitude; // config.amplitude;   /* Sensor bias voltage between reference and sense electrodes*/
	pAMPCfg->Vzero = 1100;
  pAMPCfg->Vbias = 1100;
	/* Configure Pulse*/
	pAMPCfg->pulseAmplitude = config.amplitude;						/* Pulse amplitude on counter electrode (mV) */
	pAMPCfg->pulseLength = config.ms;								/* Length of voltage pulse in ms */
}

void elec_ca_loop(void)
{
  if (!ca_is_working) return;
  uint32_t temp[n];
  AppCHRONOAMPCfg_Type *pAMPCfg;
  AppCHRONOAMPGetCfg(&pAMPCfg);
  AD5940PlatformCfg();
  pindex = 0;
  IntCount = 0;
  AD5940AMPStructInit(); /* Configure your parameters in this function */
  
  LOG_INF("CA pulse test.");
  AppCHRONOAMPInit(AppBuff[0], APPBUFF_SIZE);    /* Initialize AMP application. Provide a buffer, which is used to store sequencer commands */
  AppCHRONOAMPCtrl(CHRONOAMPCTRL_PULSETEST, 0);         /* Control AMP measurement. AMPCTRL_PULSETEST carries out pulse test*/
 
  LOG_INF("CA loop start.");
  while(ca_is_working)
  {
    /* Check if interrupt flag which will be set when interrupt occurred. */
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag(); /* Clear this flag */
      temp[IntCount] = APPBUFF_SIZE;
      AppCHRONOAMPISR(AppBuff[IntCount], &temp[IntCount]); /* Deal with it and provide a buffer to store data we got */
			if(pAMPCfg->bMeasureTransient == bFALSE)
			{
				AMPShowResult(false, (float*)AppBuff[0], temp[0]); // debug: buffer 中无数据
			}
      if(pAMPCfg->EndSeq) /* End sequence only set at end of transient */
      {
        for(int i = 0; i<IntCount; i++)
        {
          AMPShowResult(true, (float*)AppBuff[i], temp[i]); /* Show the results to UART */
        }
        pAMPCfg->EndSeq = bFALSE;
				pAMPCfg->bMeasureTransient = bFALSE;
        IntCount = 0;
        printk("Standard CA start.\n");
				AppCHRONOAMPCtrl(CHRONOAMPCTRL_START, 0); /* Begin standard amperometric measurement after pulse test is complete */
      }
    }
  }
  LOG_INF("CA loop end.");
  AppCHRONOAMPCtrl(CHRONOAMPCTRL_SHUTDOWN, 0);
  AppCHRONOAMPCtrl(CHRONOAMPCTRL_STOPSYNC, 0);
  AppCHRONOAMPCtrl(CHRONOAMPCTRL_STOPNOW, 0);
  k_msleep(10);
  uint8_t stop_signal[2];
  stop_signal[0] = 0xFF;
  stop_signal[1] = 0xFF;
  send_fac_stream_directly(stop_signal, 2);
  elec_ca_stop();
}
