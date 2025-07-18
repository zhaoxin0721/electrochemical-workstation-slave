// #include "ca.h"
// #include "../drivers/freistat_ca.h"

// #include <zephyr/kernel.h>
// #include <zephyr/logging/log.h>
// LOG_MODULE_REGISTER(elec_ca, LOG_LEVEL_INF);

// void elec_ca_loop();
// volatile bool  ca_is_working = false;
// static elec_ca_config_t config = {
//   .amplitude = 500.0f,
//   .ms = 60000,
// };


// int elec_ca_init()
// {
//   // do nothing.
//   return 0;
// }

// int elec_ca_config(elec_ca_config_t config_)
// {
//   // copy
//   config.amplitude = config_.amplitude;
//   config.ms = config_.ms;
//   return 0;
// }

// int elec_ca_start()
// {
//   // config valid?
//   ca_is_working = true;
//   // new task:
//   return elec_method_commit(&elec_ca_loop);
// }

// int elec_ca_stop()
// {
//   ca_is_working = false;
//   int err = elec_method_commit(NULL);
//   LOG_INF("CA stop: still working? %s", ca_is_working ? "yes" : "no");
//   return err;
// }

// bool elec_ca_is_working()
// {
//   return ca_is_working;
// }

// // Implemention:

// #define APPBUFF_SIZE 600
// #define n 3
// uint32_t AppBuff[n][APPBUFF_SIZE];

// float LFOSCFreq;
// uint32_t IntCount = 0;

// static uint32_t pindex;//position in dataset

// /* It's your choice here what to do with the data. Here is just an example to print to UART */
// int32_t AMPShowResult(float *pData, uint32_t DataCount)
// {
//   #define CA_DATA_NUM 1  // 30*(2+4+1+1)=240
//   #define CA_DATA_LEN (10 * CA_DATA_NUM) // 30*(2+4+1+1)=240
//   uint8_t data[CA_DATA_LEN];
//   // for(int i=0;i<DataCount;i++)
//   // {
//   //   printk("index: %d, Current: %fuA\n", pindex, pData[i]);
//   float value = pData[0];
//   int32_t uV = PulseVoltage[pindex];
//   int32_t nA = (int32_t)(value * 1000); // to nA or mV
//   int offset = pindex % CA_DATA_NUM;
//   offset *= 10;
//   data[offset++] = (uint8_t)((uV >> 24) & 0xFF);
//   data[offset++] = (uint8_t)((uV >> 16) & 0xFF);
//   data[offset++] = (uint8_t)((uV >> 8) & 0xFF);
//   data[offset++] = (uint8_t)(uV & 0xFF);
//   data[offset++] = (uint8_t)((nA >> 24) & 0xFF);
//   data[offset++] = (uint8_t)((nA >> 16) & 0xFF);
//   data[offset++] = (uint8_t)((nA >> 8) & 0xFF);
//   data[offset++] = (uint8_t)((nA) & 0xFF);
//   data[offset++] = 0x00; // V / uA
//   data[offset++] = 0x00; // ca method: I V_SE V_RE
//   pindex++;
//   if (pindex % CA_DATA_NUM == 0) {
//     k_msleep(5);
//     int err = send_fac_stream_directly(data, CA_DATA_LEN);
//     if (err) {
//       // retry--
//       k_msleep(2);
//       err = send_fac_stream_directly(data, CA_DATA_LEN);
//       if (err) {
//         LOG_ERR("Ble Face Error: %08X", err);
//       }
//     }
//   }
//     // if (is_pluse) {
//     //   break;
//     // }
//   // }
//   printk("%d: %fmV | %fuA\n", pindex + 1, PulseVoltage[pindex], pData[0]);
//   if (CurrStep != pindex) //if CurrStep chaged (after Custom interrupt 1 occured to indicated step change), from now on, the data belongs to this new step
//   {
//     pindex = CurrStep;
//   }
//   return 0;
// }

// /* Initialize AD5940 basic blocks like clock */
// static int32_t AD5940PlatformCfg(void)
// {
//   CLKCfg_Type clk_cfg;
//   FIFOCfg_Type fifo_cfg;
//   AGPIOCfg_Type gpio_cfg;
//   LFOSCMeasure_Type LfoscMeasure;

//   /* Use hardware reset */
//   AD5940_HWReset();

//   /* Platform configuration */
//   AD5940_Initialize();
//   /* Step1. Configure clock */
//   clk_cfg.ADCClkDiv = ADCCLKDIV_1;
//   clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
//   clk_cfg.SysClkDiv = SYSCLKDIV_1;
//   clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
//   clk_cfg.HfOSC32MHzMode = bFALSE;
//   clk_cfg.HFOSCEn = bTRUE;
//   clk_cfg.HFXTALEn = bFALSE;
//   clk_cfg.LFOSCEn = bTRUE;
//   AD5940_CLKCfg(&clk_cfg);
//   /* Step2. Configure FIFO and Sequencer*/
//   fifo_cfg.FIFOEn = bFALSE;
//   fifo_cfg.FIFOMode = FIFOMODE_FIFO;
//   fifo_cfg.FIFOSize = FIFOSIZE_4KB; /* 4kB for FIFO, The reset 2kB for sequencer */
//   fifo_cfg.FIFOSrc = FIFOSRC_DFT;
//   fifo_cfg.FIFOThresh = 4;   //will be overwritten in AD5940AMPStructInit()!
//   AD5940_FIFOCfg(&fifo_cfg); /* Disable to reset FIFO. */
//   fifo_cfg.FIFOEn = bTRUE;
//   AD5940_FIFOCfg(&fifo_cfg); /* Enable FIFO here */

//   /* Step3. Interrupt controller */
//   AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
//   AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
//   AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT1 | AFEINTSRC_GPT1INT_TRYBRK | AFEINTSRC_DATAFIFOOF, bTRUE); /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
//   AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
//   /* Step4: Reconfigure GPIO */
//   gpio_cfg.FuncSet = GP6_SYNC | GP5_SYNC | GP4_SYNC | GP2_TRIG | GP1_SYNC | GP0_INT;
//   gpio_cfg.InputEnSet = AGPIO_Pin2;
//   gpio_cfg.OutputEnSet = AGPIO_Pin0 | AGPIO_Pin1 | AGPIO_Pin4 | AGPIO_Pin5 | AGPIO_Pin6;
//   gpio_cfg.OutVal = AGPIO_Pin1; //set high to turn off LED
//   gpio_cfg.PullEnSet = 0;
//   AD5940_AGPIOCfg(&gpio_cfg);

//   AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); /* Enable AFE to enter sleep mode. */
//   /* Measure LFOSC frequency */
//   LfoscMeasure.CalDuration = 1000.0; /* 1000ms used for calibration. */
//   LfoscMeasure.CalSeqAddr = 0;
//   LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
//   AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);

// #ifdef DEBUG
//   printk("Measured LFOSC Freq (used for sequencer timing) - rounded: %d Hz\n", (int)(LFOSCFreq + 0.5));
// #endif
//   return 0;
// }

// /* !!Change the application parameters here if you want to change it to none-default value */
// void AD5940AMPStructInit(void)
// {
//   AppCHRONOAMPCfg_Type *pAMPCfg;
//   AppCHRONOAMPGetCfg(&pAMPCfg);
//   /* Configure general parameters */
//   pAMPCfg->WuptClkFreq = LFOSCFreq; /* Use measured 32kHz clock freq for accurate wake up timer */
//   pAMPCfg->SeqStartAddr = 0;
//   pAMPCfg->MaxSeqLen = 512; /* 2kB/4BytesPerCommand = 512 sequencer commands */
//   pAMPCfg->RcalVal = 1000.0;

//   pAMPCfg->FifoThresh = 10; //determines how many samples get averaged for output!!!
//   pAMPCfg->ADCRefVolt = 1.82; /* The real ADC reference voltage. Measure it from capacitor C3 (AD5941 FeatherWing Rev1 Board) with DMM. */

//   pAMPCfg->ExtRtia = bFALSE;              /* Set to true if using external Rtia */
//   pAMPCfg->ExtRtiaVal = 10000000;         /* Enter external Rtia value here if using one */
//   pAMPCfg->LptiaRtiaSel = LPTIARTIA_8K; /* Select TIA gain resistor. */

//   pAMPCfg->SensorBias = config.amplitude; /* Sensor bias voltage between sense and reference electrodes in mV*/
//   pAMPCfg->Vzero = 1300;   /* Set potential at WE, determines the positive and negative range */

//   pAMPCfg->LpAmpPwrMod = LPAMPPWR_NORM;  //LPAMPPWR_BOOST3;/* restrict to max. +/- 750 uA cell current*/
  
//   /* Configure Pulse*/
//   // int numCycles = 1; //number of cycles
//   pAMPCfg->StepNumber = 1; // 3 * numCycles;  //each cycle has 3 pulses
//   pAMPCfg->pulseAmplitude[0]  = config.amplitude;/* Pulse amplitude (WE - RE) (mV) */
//   pAMPCfg->pulseLength[0]     = config.ms;/* Length of voltage pulse in ms */
//   // uint32_t each_ms = config.ms / 3 + 1;
//   // //fill the pulseAmplitude and pulseLength arrays with the voltages for 3 pulses per cycle
//   // for (int i = 0; i < numCycles; i++)
//   // {
//   //   pAMPCfg->pulseAmplitude[i]    = config.amplitude; /* Pulse amplitude (WE - RE) (mV) */
//   //   // pAMPCfg->pulseAmplitude[i*3+1]  = config.amplitude;
//   //   // pAMPCfg->pulseAmplitude[i*3+2]  = config.amplitude;
//   //   pAMPCfg->pulseLength[i]   = each_ms; /* Length of voltage pulse in ms */
//   //   // pAMPCfg->pulseLength[i*3+1] = each_ms;
//   //   // pAMPCfg->pulseLength[i*3+2] = each_ms;
//   // }
  
//   /* ADC Configuration*/
//   pAMPCfg->ADCPgaGain = ADCPGA_1P5;
//   pAMPCfg->ADCSinc3Osr = ADCSINC3OSR_2; //Only change according to table 41 in datasheet to use 50Hz filter
//   pAMPCfg->ADCSinc2Osr = ADCSINC2OSR_1333;  //Only change according to table 41 in datasheet to use 50Hz filter
// }

// void elec_ca_loop(void)
// {
//   // init.
//   CurrStep = 0;
//   pindex = 0;

//   // original init.
//   uint32_t temp;
//   AppCHRONOAMPCfg_Type *pAMPCfg;
//   AppCHRONOAMPGetCfg(&pAMPCfg);
//   AD5940PlatformCfg();

//   AD5940AMPStructInit(); /* Configure your parameters in this function */

//   AppCHRONOAMPInit(AppBuff, APPBUFF_SIZE); /* Initialize AMP application. Provide a buffer, which is used to store sequencer commands */

// //after application init, print the configuration parameters
// #ifdef DEBUG
//   int Sinc3OSR[] = {5, 4, 2};
//   int Sinc2OSR[] = {22, 44, 89, 178, 267, 533, 640, 667, 800, 889, 1067, 1333};
//   float PgaGain[] = {1, 1.5, 2, 4, 9};
//   printk("---Configuration of CA---\n");
//   //conifgured pulse voltages
//   printk("Config of Pulses (integer values):\n");
//   for (int i = 0; i < pAMPCfg->StepNumber; i++)
//   {
//     printk("%u: %d mV - %u ms\n", (i + 1), (int)pAMPCfg->pulseAmplitude[i], (uint16_t)pAMPCfg->pulseLength[i]);
//   }
//   //exact pulse voltages
//   printk("DAC outputs (integer values):\n");
//   for (int i = 0; i < pAMPCfg->StepNumber; i++)
//   {
//     printk("%u: %d uV\n", (i + 1), (int)(PulseVoltage[i] * 1000));
//   }

//   //OSR
//   printk("Sinc3OSR: %u\n", Sinc3OSR[pAMPCfg->ADCSinc3Osr]);
//   printk("Sinc2OSR: %u\n", Sinc2OSR[pAMPCfg->ADCSinc2Osr]);
//   //PGA Gain
//   printk("ADC-PGA-Gain = %d.", ((int)PgaGain[pAMPCfg->ADCPgaGain]));
//   printk("%d\n", (int)(PgaGain[pAMPCfg->ADCPgaGain] * 10.0 - ((int)PgaGain[pAMPCfg->ADCSinc3Osr] * 10)));
//   //Calibrated Rtia
//   printk("Calibrated Rtia - Magnitude = %d.", ((int)pAMPCfg->RtiaCalValue.Magnitude));
//   int digits = (int)((pAMPCfg->RtiaCalValue.Magnitude * 1000.0 - ((int)pAMPCfg->RtiaCalValue.Magnitude * 1000)) + 0.0005);
//   if (digits < 100)
//     printk("0"); //one leading 0
//   if (digits < 10)
//     printk("0"); //two leading 0
//   printk("%d Ohm\n", digits);
//   //Calibrated Rtia
//   //printk("Calibrated Rtia - Magnitude = %d.", ((int)pAMPCfg->RtiaCalValue.Magnitude));
//   //printk("%d Ohm\n", (int)((pAMPCfg->RtiaCalValue.Magnitude * 1000.0 - ((int)pAMPCfg->RtiaCalValue.Magnitude * 1000)) + 0.0005));
//   //current range
//   //check different ADC input voltage ranges for different PGA settings
//   if (PgaGain[pAMPCfg->ADCPgaGain] <= 1.5) //PGA = 1, 1.5
//     printk("Current range: +/- %d uA\n", (int)(900.0 / PgaGain[pAMPCfg->ADCPgaGain] / pAMPCfg->RtiaCalValue.Magnitude * 1000 + 0.5));
//   else if (PgaGain[pAMPCfg->ADCPgaGain] == 2) //PGA = 2
//     printk("Current range: +/- %d uA\n", (int)(600.0 / PgaGain[pAMPCfg->ADCPgaGain] / pAMPCfg->RtiaCalValue.Magnitude * 1000 + 0.5));
//   else if (PgaGain[pAMPCfg->ADCPgaGain] == 4) //PGA = 4
//     printk("Current range: +/- %d uA\n", (int)(300.0 / PgaGain[pAMPCfg->ADCPgaGain] / pAMPCfg->RtiaCalValue.Magnitude * 1000 + 0.5));
//   else if (PgaGain[pAMPCfg->ADCPgaGain] == 9) //PGA = 9
//     printk("Current range: +/- %d uA\n", (int)(133.0 / PgaGain[pAMPCfg->ADCPgaGain] / pAMPCfg->RtiaCalValue.Magnitude * 1000 + 0.5));
//   //resolution
//   float resolution = pAMPCfg->ADCRefVolt / (32768 * pAMPCfg->RtiaCalValue.Magnitude * PgaGain[pAMPCfg->ADCPgaGain]) * 1E9; //resolution in nA
//   if (resolution < 1)
//   {
//     //output resolution in pA
//     resolution *= 1E3; //convert to pA
//     printk("Current resolution: %d pA\n", (int)resolution);
//   }
//   //output resolution in nA
//   else
//   {
//     printk("Current resolution: %d nA\n", (int)resolution);
//   }
// #endif

// //send CA config data to python application
// #ifdef PLOT_DATA
//   AppCHRONOAMPGetCfg(&pAMPCfg);
// #ifndef DEBUG
//   int Sinc3OSR[] = {5, 4, 2};
//   int Sinc2OSR[] = {22, 44, 89, 178, 267, 533, 640, 667, 800, 889, 1067, 1333};
// #endif
//   uint8_t byteData[30]; //buffer for sending bytes to python application

//   //send exact pulse voltages (in mV)
//   uint8_t *pulseBytes = (uint8_t *)PulseVoltage; //cast pointer to the exact pulse voltage array to a byte pointer
//   //write  all the bytes to serial port
//   Serial.write(pulseBytes, (MAX_STEPS * 4)); //each entry in PulseVoltage has 4 bytes (float), MAX_STEPS entries

//   //calculate duration of CA (needed to set x axis limits)
//   uint16_t CA_duration = 0;
//   for (int i = 0; i < pAMPCfg->StepNumber; i++)
//   {
//     CA_duration += (int) pAMPCfg->pulseLength[i]/1000+0.5;
//   }

//   //pack  important config data into buffer
//   byteData[0] = ((int16_t)pAMPCfg->StepNumber & 0xFF00) >> 8;
//   byteData[1] = (int16_t)pAMPCfg->StepNumber & 0x00FF;
//   byteData[2] = ((int16_t)Sinc3OSR[pAMPCfg->ADCSinc3Osr] & 0xFF00) >> 8;
//   byteData[3] = (int16_t)Sinc3OSR[pAMPCfg->ADCSinc3Osr] & 0x00FF;
//   byteData[4] = ((int16_t)Sinc2OSR[pAMPCfg->ADCSinc2Osr] & 0xFF00) >> 8;
//   byteData[5] = (int16_t)Sinc2OSR[pAMPCfg->ADCSinc2Osr] & 0x00FF;
//   byteData[6] = (CA_duration & 0xFF00) >> 8;
//   byteData[7] = CA_duration & 0x00FF;

//   //write the bytes to serial port
//   Serial.write(&byteData[0], 8);

//   //delay to allow python to set everything up
//   delay(3000);
// #endif

// #ifdef DEBUG
//   printk("---start of CA---\n");
// #endif

//   AppCHRONOAMPCtrl(CHRONOAMPCTRL_PULSETEST, 0); /* Control AMP measurement. AMPCTRL_PULSETEST carries out pulse test*/

//   while (1)
//   {
//     /* Check if interrupt flag which will be set when interrupt occurred. */
//     if (AD5940_GetMCUIntFlag())
//     {
//       AD5940_ClrMCUIntFlag(); /* Clear this flag */
//       temp = APPBUFF_SIZE;
//       AppCHRONOAMPISR(AppBuff, &temp); /* Deal with it and provide a buffer to store data we got */

//       AMPShowResult((float *)AppBuff, temp);

//       if (pAMPCfg->EndSeq)
//       {

// #ifdef DEBUG
//         printk( "---end of CA---\n");
// #endif
//         while (1)
//           ;
//       }
//     }
//   }
//   k_msleep(10);
//   uint8_t stop_signal[2];
//   stop_signal[0] = 0xFF;
//   stop_signal[1] = 0xFF;
//   send_fac_stream_directly(stop_signal, 2);
//   elec_ca_stop();
// }
