// #include "cv.h"
// #include "../drivers/cyclic_voltammetry.h"

// #include <zephyr/kernel.h>
// #include <zephyr/logging/log.h>
// LOG_MODULE_REGISTER(elec_cv, LOG_LEVEL_INF);

// void elec_cv_loop();
// static volatile bool working = false;
// static elec_cv_config_t config = {
//   .start_volt   = -1000.0f,           /* -1V */
//   .peak_volt    = +1000.0f,           /* +1V */
//   .vzero_start  = 2200.0f,            /* 1.3V */
//   .vzero_peak   = 400.0f,             /* 1.3V */
//   .sample_delay = 10.0f,               /* 7ms. Time between update DAC and ADC sample. Unit is ms. */
//   .step_number  = 1000,                /* Total steps. Equals to ADC sample number */
//   .duration     = 40*1000,            /* X * 1000, where x is total duration of ramp signal. Unit is ms. */
//   .repeat_number = 0,
// };
// static uint32_t index;

// int elec_cv_init()
// {
//   return 0;
// }

// int elec_cv_config(elec_cv_config_t config_)
// {
//   config.start_volt   = config_.start_volt;
//   config.peak_volt    = config_.peak_volt;
//   config.vzero_start  = config_.vzero_start ;
//   config.vzero_peak   = config_.vzero_peak;
//   config.sample_delay = config_.sample_delay;
//   config.step_number  = config_.step_number;
//   config.duration     = config_.duration;
//   config.repeat_number = config_.repeat_number;
//   return 0;
// }

// int elec_cv_start()
// {
//   index = 0;
//   working = true;
//   return elec_method_commit(&elec_cv_loop);
// }

// int elec_cv_stop()
// {
//   index = 0;
//   working = false;
//   int err = elec_method_commit(NULL);
//   LOG_INF("CV stop: still working? %s", working ? "yes" : "no");
//   return err;
// }

// bool elec_cv_is_working()
// {
//   return working;
// }

// // Implementation:

// #define CVAPPBUFF_SIZE 1024
// uint32_t CVAppBuff[CVAPPBUFF_SIZE];
// float CVLFOSCFreq;    /* Measured LFOSC frequency */

// /**
//  * @brief An example to deal with data read back from AD5940. Here we just print data to UART
//  * @note UART has limited speed, it has impact when sample rate is fast. Try to print some of the data not all of them.
//  * @param pData: the buffer stored data for this application. The data from FIFO has been pre-processed.
//  * @param DataCount: The available data count in buffer pData.
//  * @return return 0.
// */
// static int32_t RampShowResult(float *pData, uint32_t DataCount)
// {
//   /* Print data*/
//   int err = 0;
//   uint8_t data[18]; // 3*(2+4)=18
//   for(int i=0;i<DataCount;i++)
//   {
//     if (i % 100 == 0) { // filter.
//       LOG_INF("index:%d, %.3f uA", index, pData[i]);
//     }
//     //i += 10;  /* Print though UART consumes too much time. */
//     uint16_t id = index;
//     int32_t nA = (int32_t)(pData[i] * 1000); // to nA
//     int offset = index % 3;
//     offset *= 6;
//     data[offset] = (uint8_t)((id >> 8) & 0xFF);
//     data[offset + 1] = (uint8_t)(id & 0xFF);
//     data[offset + 2] = (uint8_t)((nA >> 24) & 0xFF);
//     data[offset + 3] = (uint8_t)((nA >> 16) & 0xFF);
//     data[offset + 4] = (uint8_t)((nA >> 8) & 0xFF);
//     data[offset + 5] = (uint8_t)((nA) & 0xFF);
//     if (index % 3 == 2) {
//       k_msleep(5);
//       err = send_fac_stream_directly(data, 18);
//       if (err) {
//         // retry--
//         k_msleep(2);
//         err = send_fac_stream_directly(data, 18);
//         if (err) {
//           LOG_ERR("Ble Face Error: %08X", err);
//         }
//       }
//     }
//     index++;
//   }
//   return 0;
// }

// /**
//  * @brief The general configuration to AD5940 like FIFO/Sequencer/Clock. 
//  * @note This function will firstly reset AD5940 using reset pin.
//  * @return return 0.
// */
// static int32_t AD5940PlatformCfg(void)
// {
//   CLKCfg_Type clk_cfg;
//   SEQCfg_Type seq_cfg;  
//   FIFOCfg_Type fifo_cfg;
//   AGPIOCfg_Type gpio_cfg;
//   LFOSCMeasure_Type LfoscMeasure;

//   /* Use hardware reset */
//   AD5940_HWReset();
//   AD5940_Initialize();    /* Call this right after AFE reset */
//   /* Platform configuration */
//   /* Step1. Configure clock */
//   clk_cfg.HFOSCEn = bTRUE;
//   clk_cfg.HFXTALEn = bFALSE;
//   clk_cfg.LFOSCEn = bTRUE;
//   clk_cfg.HfOSC32MHzMode = bFALSE;
//   clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
//   clk_cfg.SysClkDiv = SYSCLKDIV_1;
//   clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
//   clk_cfg.ADCClkDiv = ADCCLKDIV_1;
//   AD5940_CLKCfg(&clk_cfg);
//   /* Step2. Configure FIFO and Sequencer*/
//   /* Configure FIFO and Sequencer */
//   fifo_cfg.FIFOEn = bTRUE;           /* We will enable FIFO after all parameters configured */
//   fifo_cfg.FIFOMode = FIFOMODE_FIFO;
//   fifo_cfg.FIFOSize = FIFOSIZE_2KB;   /* 2kB for FIFO, The reset 4kB for sequencer */
//   fifo_cfg.FIFOSrc = FIFOSRC_SINC3;   /* */
//   fifo_cfg.FIFOThresh = 4;            /*  Don't care, set it by application paramter */
//   AD5940_FIFOCfg(&fifo_cfg);
//   seq_cfg.SeqMemSize = SEQMEMSIZE_4KB;  /* 4kB SRAM is used for sequencer, others for data FIFO */
//   seq_cfg.SeqBreakEn = bFALSE;
//   seq_cfg.SeqIgnoreEn = bTRUE;
//   seq_cfg.SeqCntCRCClr = bTRUE;
//   seq_cfg.SeqEnable = bFALSE;
//   seq_cfg.SeqWrTimer = 0;
//   AD5940_SEQCfg(&seq_cfg);
//   /* Step3. Interrupt controller */
//   AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
//   AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
//   AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH|AFEINTSRC_ENDSEQ|AFEINTSRC_CUSTOMINT0, bTRUE); 
//   AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
//   /* Step4: Configure GPIO */
//   gpio_cfg.FuncSet = GP0_INT|GP1_SLEEP|GP2_SYNC;  /* GPIO1 indicates AFE is in sleep state. GPIO2 indicates ADC is sampling. */
//   gpio_cfg.InputEnSet = 0;
//   gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2;
//   gpio_cfg.OutVal = 0;
//   gpio_cfg.PullEnSet = 0;
//   AD5940_AGPIOCfg(&gpio_cfg);
//   /* Measure LFOSC frequency */
//   /**@note Calibrate LFOSC using system clock. The system clock accuracy decides measurement accuracy. Use XTAL to get better result. */
//   LfoscMeasure.CalDuration = 1000.0;  /* 1000ms used for calibration. */
//   LfoscMeasure.CalSeqAddr = 0;        /* Put sequence commands from start address of SRAM */
//   LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
//   AD5940_LFOSCMeasure(&LfoscMeasure, &CVLFOSCFreq);
//   printk("LFOSC Freq:%f\n", CVLFOSCFreq);
//   AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);         /*  */
//   return 0;
// }

// /**
//  * @brief The interface for user to change application paramters.
//  * @return return 0.
// */
// void AD5940RampStructInit(void)
// {
//   AppRAMPCfg_Type *pRampCfg;
  
//   AppRAMPGetCfg(&pRampCfg);
//   /* Step1: configure general parmaters */
//   pRampCfg->SeqStartAddr = 0x10;                /* leave 16 commands for LFOSC calibration.  */
//   pRampCfg->MaxSeqLen = 1024-0x10;              /* 4kB/4 = 1024  */
//   pRampCfg->RcalVal = 1000.0;                   /* 10kOhm RCAL */
//   pRampCfg->ADCRefVolt = 1820.0f;               /* The real ADC reference voltage. Measure it from capacitor C12 with DMM. */
//   pRampCfg->FifoThresh = 480;                   /* Maximum value is 2kB/4-1 = 512-1. Set it to higher value to save power. */
//   pRampCfg->SysClkFreq = 16000000.0f;           /* System clock is 16MHz by default */
//   pRampCfg->LFOSCClkFreq = CVLFOSCFreq;           /* LFOSC frequency */
//   /* Configure ramp signal parameters */
//   pRampCfg->RampStartVolt = config.start_volt; //  -1000.0f;           /* -1V */
//   pRampCfg->RampPeakVolt = config.peak_volt; // +1000.0f;           /* +1V */
//   pRampCfg->VzeroStart = config.vzero_start; // 1300.0f;               /* 1.3V */
//   pRampCfg->VzeroPeak = config.vzero_peak; // 1300.0f;                /* 1.3V */
//   pRampCfg->StepNumber = config.step_number; // 800;                   /* Total steps. Equals to ADC sample number */
//   pRampCfg->RampDuration = config.duration; // 24*1000;            /* X * 1000, where x is total duration of ramp signal. Unit is ms. */
//   pRampCfg->SampleDelay = config.sample_delay; // 7.0f;                 /* 7ms. Time between update DAC and ADC sample. Unit is ms. */
//   pRampCfg->LPTIARtiaSel = LPTIARTIA_4K;       /* Maximum current decides RTIA value */
// 	pRampCfg->LPTIARloadSel = LPTIARLOAD_SHORT;
// 	pRampCfg->AdcPgaGain = ADCPGA_1P5;
// }

// void elec_cv_loop()
// {
//   uint32_t temp; 
//   AppRAMPCfg_Type *pRampCfg;	
//   AD5940PlatformCfg();
//   AD5940RampStructInit();

//   AppRAMPInit(CVAppBuff, CVAPPBUFF_SIZE);    /* Initialize RAMP application. Provide a buffer, which is used to store sequencer commands */
//   AppRAMPCtrl(APPCTRL_START, 0);          /* Control IMP measurement to start. Second parameter has no meaning with this command. */

//   index = 0;
//   int16_t repeat = config.repeat_number;
//   LOG_INF("CV working? %s", working ? "yes" : "no");
//   while(working)
//   {
// 		AppRAMPGetCfg(&pRampCfg);
//     if(AD5940_GetMCUIntFlag())
//     {
//       AD5940_ClrMCUIntFlag();
//       temp = CVAPPBUFF_SIZE;
//       AppRAMPISR(CVAppBuff, &temp);
//       RampShowResult((float*)CVAppBuff, temp);
//     }
// 		/* Repeat Measurement continuously*/
// 		if(pRampCfg->bTestFinished ==bTRUE)
// 		{
//       if (repeat <= 0) {
//         index = 0;
// 			  pRampCfg->bTestFinished = bFALSE;
//         LOG_INF("CV Process exit.");
//         break;
//       }
//       repeat--;
//       index = 0;
//       LOG_INF("Repeat once, rest: %d", repeat);
// 			AD5940_Delay10us(200000);
// 			pRampCfg->bTestFinished = bFALSE;
//       // start new one:
// 			AD5940_SEQCtrlS(bTRUE);   /* Enable sequencer, and wait for trigger */
// 			AppRAMPCtrl(APPCTRL_START, 0);
//       uint8_t cycle_end_signal[2];
//       cycle_end_signal[0] = 0xEE;
//       cycle_end_signal[1] = 0xEE;
//       send_fac_stream_directly(cycle_end_signal, 2);
// 		}
//   }
//   AppIMPCtrl(APPCTRL_STOPNOW, 0);
//   k_msleep(10);
//   uint8_t stop_signal[2];
//   stop_signal[0] = 0xFF;
//   stop_signal[1] = 0xFF;
//   send_fac_stream_directly(stop_signal, 2);
//   elec_cv_stop();
// }
