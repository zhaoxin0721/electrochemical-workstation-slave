#include "custom_cv.h"
#include <zephyr/kernel.h>
// #include "math.h"
#define ABS(a) ((a) < (0) ? -(a) : (a))

#define DAC12BITVOLT_1LSB   (2200.0f/4095)  //mV
#define DAC6BITVOLT_1LSB    (DAC12BITVOLT_1LSB*64)  //mV

#define LPTIARTIA_SEL LPTIARTIA_4K
#define ADCPGA_Gain   ADCPGA_1P5
#define ADCRef_Volt   1820.0f
#define ADCSinc3OSR   ADCSINC3OSR_4
#define ADC_Clk_Freq  (16000000.0f)
#define R_Cal_Val     (1000.0f)

typedef struct {
  fImpPol_Type  r_tia_val;
  float         CVLFOSC_freq;
  ccv_params_t  params;
  int           current_step;
  int           invert_step; // 电压变换方向处
  float         slope_k;      // 斜率
  float         volt_per_step; // 每步行走电压
} state_;

static state_ state = {
  .r_tia_val    = R_Cal_Val,
};

// 校准 Rtia
static AD5940Err rtia_calibration(void)
    {
    fImpPol_Type RtiaCalValue;  /* Calibration result */
    LPRTIACal_Type lprtia_cal;
    AD5940_StructInit(&lprtia_cal, sizeof(lprtia_cal));

    lprtia_cal.LpAmpSel = LPAMP0;
    lprtia_cal.bPolarResult = bTRUE;                /* Magnitude + Phase */
    lprtia_cal.AdcClkFreq = ADC_Clk_Freq;  
    lprtia_cal.SysClkFreq = 16000000.0f;            /* System clock is 16MHz by default */
    lprtia_cal.ADCSinc3Osr = ADCSINC3OSR_4;
    lprtia_cal.ADCSinc2Osr = ADCSINC2OSR_22;        /* Use SINC2 data as DFT data source */
    lprtia_cal.DftCfg.DftNum = DFTNUM_2048;         /* Maximum DFT number */
    lprtia_cal.DftCfg.DftSrc = DFTSRC_SINC2NOTCH;
    lprtia_cal.DftCfg.HanWinEn = bTRUE;
    lprtia_cal.fFreq = ADC_Clk_Freq / 4 / 22 / 2048 * 3; /* Sample 3 period of signal, 13.317Hz here. Do not use DC method, because it needs ADC/PGA calibrated firstly(but it's faster) */
    lprtia_cal.fRcal = R_Cal_Val;
    lprtia_cal.LpTiaRtia = LPTIARTIA_SEL;
    lprtia_cal.LpAmpPwrMod = LPAMPPWR_NORM;
    lprtia_cal.bWithCtia = bFALSE;
    AD5940_LPRtiaCal(&lprtia_cal, &RtiaCalValue);
    state.r_tia_val = RtiaCalValue;
    //printf("Rtia,%f,%f\n", RtiaCalValue.Magnitude, RtiaCalValue.Phase);
    return AD5940ERR_OK;
}

// 初始化芯片配置
int ccv_init(ccv_params_t params)
{
  // state.params = params;
  state.params.start_volt = params.start_volt;
  state.params.peak_volt = params.peak_volt;
  state.params.vzero_start = params.vzero_start;
  state.params.vzero_peak = params.vzero_peak;
  state.params.sample_delay = params.sample_delay;
  state.params.step_number = params.step_number;
  state.params.duration = params.duration;
  state.params.repeat_number = params.repeat_number;

  AD5940Err error = AD5940ERR_OK;

  CLKCfg_Type clk_cfg;
  AGPIOCfg_Type gpio_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  const uint32_t *pSeqCmd;
  uint32_t SeqLen;
  AFERefCfg_Type aferef_cfg;
  LPLoopCfg_Type lploop_cfg;
  DSPCfg_Type dsp_cfg;
  HSLoopCfg_Type hs_loop;

  // platform init.

  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize();    /* Call this right after AFE reset */

  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);

  /* Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH|AFEINTSRC_ENDSEQ|AFEINTSRC_CUSTOMINT0, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Configure GPIO */
  gpio_cfg.FuncSet = GP0_INT|GP1_SLEEP|GP2_SYNC;  /* GPIO1 indicates AFE is in sleep state. GPIO2 indicates ADC is sampling. */
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  /* Measure LFOSC frequency */
  /**@note Calibrate LFOSC using system clock. The system clock accuracy decides measurement accuracy. Use XTAL to get better result. */
  LfoscMeasure.CalDuration = 1000.0;  /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;        /* Put sequence commands from start address of SRAM */
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, &(state.CVLFOSC_freq));
  // app init.

  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);  /* Init all to disable state */

  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;
  aferef_cfg.Lp1V1BuffEn = bFALSE;
  aferef_cfg.Lp1V8BuffEn = bFALSE;
  /* LP reference control - turn off them to save power*/
  aferef_cfg.LpBandgapEn = bTRUE;
  aferef_cfg.LpRefBufEn = bTRUE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);
  
  lploop_cfg.LpAmpCfg.LpAmpSel = LPAMP0;
  lploop_cfg.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_BOOST3;
  lploop_cfg.LpAmpCfg.LpPaPwrEn = bTRUE;
  lploop_cfg.LpAmpCfg.LpTiaPwrEn = bTRUE;
  lploop_cfg.LpAmpCfg.LpTiaRf = LPTIARF_20K;
  lploop_cfg.LpAmpCfg.LpTiaRload = LPTIARLOAD_SHORT; // roumai: 2023-12-19
  lploop_cfg.LpAmpCfg.LpTiaRtia = LPTIARTIA_SEL;
  lploop_cfg.LpAmpCfg.LpTiaSW = LPTIASW(2)|LPTIASW(4)|LPTIASW(5)|LPTIASW(9); // |LPTIASW(12);
  lploop_cfg.LpDacCfg.LpdacSel = LPDAC0;
  lploop_cfg.LpDacCfg.DacData12Bit = 0x800;
  lploop_cfg.LpDacCfg.DacData6Bit = 0;
  lploop_cfg.LpDacCfg.DataRst = bFALSE;
  lploop_cfg.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA/*|LPDACSW_VBIAS2PIN*/ | LPDACSW_VZERO2LPTIA/*|LPDACSW_VZERO2PIN*/;
  lploop_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lploop_cfg.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
  lploop_cfg.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT; /* Step Vbias. Use 12bit DAC ouput */
  lploop_cfg.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;  /* Base is Vzero. Use 6 bit DAC ouput */
  lploop_cfg.LpDacCfg.PowerEn = bTRUE;
  AD5940_LPLoopCfgS(&lploop_cfg);

  AD5940_StructInit(&dsp_cfg, sizeof(dsp_cfg));
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_LPTIA0_N;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_LPTIA0_P;
  dsp_cfg.ADCBaseCfg.ADCPga = ADCPGA_Gain;

  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = ADCSinc3OSR;
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;  /* ADC runs at 16MHz clock in this example, sample rate is 800kHz */
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;        /* We use data from SINC3 filter */
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = ADCSINC2OSR_22;  /* Don't care */
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_2;   /* Don't care because it's disabled */
  AD5940_DSPCfgS(&dsp_cfg);

  // updated: 2023-12-15, sw
  hs_loop.SWMatCfg.Dswitch = SWD_CE0;
  hs_loop.SWMatCfg.Pswitch = SWP_RE0;
  hs_loop.SWMatCfg.Nswitch = SWN_SE0;
  hs_loop.SWMatCfg.Tswitch = SWT_OPEN;
  // AD5940_HSLoopCfgS(&hs_loop);

  rtia_calibration();

  return AD5940ERR_OK;
}

static int ccv_dac_update(float * volt_mV)
{
  // AD5940_WakeUp(10);
  // Calculate current volt for Vbias & Vzero
  uint32_t VbiasCode, VzeroCode;
  if (state.current_step < state.invert_step) {
    VzeroCode = (uint32_t)((state.params.vzero_start - 200.0f) / DAC6BITVOLT_1LSB);
    float volt = state.params.start_volt + state.current_step * state.slope_k; // a+kt
    *volt_mV = volt;
    VbiasCode = volt / DAC12BITVOLT_1LSB;
  } else {
    VzeroCode = (uint32_t)((state.params.vzero_peak - 200.0f) / DAC6BITVOLT_1LSB);
    float volt = 2 * state.params.peak_volt - state.params.start_volt - state.current_step * state.slope_k; // 2b-a+kt
    *volt_mV = volt;
    VbiasCode = volt / DAC12BITVOLT_1LSB;
  }
  VzeroCode = (uint32_t)1100.0f / DAC6BITVOLT_1LSB; // reset to 1.1V.
  VbiasCode = (uint32_t)(VzeroCode * 64 + VbiasCode);
  AD5940_WriteReg(REG_AFE_LPDACDAT0, VzeroCode << 12 | VbiasCode);
  // need 10 clk ~ 1us to update data. Before send AFE to sleep state, wait 10 extra clocks
  k_usleep(1);
  // go to sleep mode.
  // AD5940_EnterSleepS();
  return 0;
}

static int ccv_adc_rtia_read(float * current_uA)
{
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);                       /* Start ADC */
  k_usleep(250);                                                /* wait 250us for reference power up */
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);                       /* Start ADC conversion before applying step to capture peak */
  uint32_t code = AD5940_ReadAfeResult(AFERESULT_SINC3);
  float volt = -AD5940_ADCCode2Volt(code, ADCPGA_Gain, ADCRef_Volt);
  *current_uA = volt / state.r_tia_val.Magnitude * 1e3f;
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV, bFALSE);       /* Stop ADC */
  return 0;
}

int ccv_start(ccv_callback_t ccv_callback)
{
  float volt_mV;
  float current_uA;
  // calculate `invert_step`, `one_step_delay`:
  uint32_t one_step_delay = state.params.duration / state.params.step_number - 1;
  state.invert_step   = state.params.step_number / 2;
  int volt_diff       = state.params.peak_volt - state.params.start_volt;
  state.slope_k       = 2 * volt_diff / state.params.step_number;
  state.volt_per_step = ABS(volt_diff) / state.invert_step;
  // init.
  state.current_step  = 0;
  int repeat = state.params.repeat_number;
  while(true) {
    if (state.current_step > state.params.step_number) {
      if (repeat > 0) repeat--;
      else break;
    }
    printk("Loop %d\t", state.current_step);
    ccv_dac_update(&volt_mV); // one loop
    k_msleep(one_step_delay > 0 ? one_step_delay : 1);
    ccv_adc_rtia_read(&current_uA);
    printk("Volt: %fmV, current: %fuA\n", volt_mV, current_uA);
    // send to ble.
    if (ccv_callback != NULL) {
      ccv_callback(volt_mV, current_uA);
    }
    state.current_step++;
  }
  // turn off DAC, ADC.
  return 0;
}
