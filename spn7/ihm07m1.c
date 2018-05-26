// ihm07m1.c
#include "ihm07m1.h"
#include "6StepLib.h"

extern SIXSTEP_Base_InitTypeDef SIXSTEP_parameters;
extern SIXSTEP_PI_PARAM_InitTypeDef_t PI_parameters;

//{{{
void bemfDelayCalc() {

 if (PI_parameters.Reference >= 0) {
   if(SIXSTEP_parameters.speed_fdbk_filtered<=12000 && SIXSTEP_parameters.speed_fdbk_filtered>10000)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_1;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=10000 && SIXSTEP_parameters.speed_fdbk_filtered>7800)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_2;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=7800 && SIXSTEP_parameters.speed_fdbk_filtered>6400)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_3;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=6400 && SIXSTEP_parameters.speed_fdbk_filtered>5400)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_4;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=5400 && SIXSTEP_parameters.speed_fdbk_filtered>4650)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_5;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=4650 && SIXSTEP_parameters.speed_fdbk_filtered>4100)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_6;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=4100 && SIXSTEP_parameters.speed_fdbk_filtered>3650)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_7;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=3650 && SIXSTEP_parameters.speed_fdbk_filtered>3300)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_8;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=3300 && SIXSTEP_parameters.speed_fdbk_filtered>2600)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_9;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=2600 && SIXSTEP_parameters.speed_fdbk_filtered>1800)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_10;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=1800 && SIXSTEP_parameters.speed_fdbk_filtered>1500)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_11;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=1500 && SIXSTEP_parameters.speed_fdbk_filtered>1300)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_12;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=1300 && SIXSTEP_parameters.speed_fdbk_filtered>1000)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_13;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=1000 && SIXSTEP_parameters.speed_fdbk_filtered>500)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_14;
     }
   else {
    if(SIXSTEP_parameters.speed_fdbk_filtered>=-12000 && SIXSTEP_parameters.speed_fdbk_filtered<-10000)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_1;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-10000 && SIXSTEP_parameters.speed_fdbk_filtered<-7800)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_2;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-7800 && SIXSTEP_parameters.speed_fdbk_filtered<-6400)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_3;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-6400 && SIXSTEP_parameters.speed_fdbk_filtered<-5400)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_4;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-5400 && SIXSTEP_parameters.speed_fdbk_filtered<-4650)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_5;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-4650 && SIXSTEP_parameters.speed_fdbk_filtered<-4100)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_6;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-4100 && SIXSTEP_parameters.speed_fdbk_filtered<-3650)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_7;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-3650 && SIXSTEP_parameters.speed_fdbk_filtered<-3300)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_8;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-3300 && SIXSTEP_parameters.speed_fdbk_filtered<-2650)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_9;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-2600 && SIXSTEP_parameters.speed_fdbk_filtered<-1800)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_10;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1800 && SIXSTEP_parameters.speed_fdbk_filtered<-1500)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_11;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1500 && SIXSTEP_parameters.speed_fdbk_filtered<-1300)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_12;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1300 && SIXSTEP_parameters.speed_fdbk_filtered<-1000)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_13;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1000 && SIXSTEP_parameters.speed_fdbk_filtered<-500)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_14;
    }
  }
//}}}

//{{{
void MC_Nucleo_Init() {

  // TIM ETR CONFIGURATION
  TIM_ClearInputConfigTypeDef sClearInputConfig;
  sClearInputConfig.ClearInputState = 1;
  sClearInputConfig.ClearInputSource = TIM_CLEARINPUTSOURCE_ETR;
  sClearInputConfig.ClearInputPolarity = TIM_CLEARINPUTPOLARITY_NONINVERTED;
  sClearInputConfig.ClearInputPrescaler = TIM_CLEARINPUTPRESCALER_DIV1;
  sClearInputConfig.ClearInputFilter = 0;
  HAL_TIM_ConfigOCrefClear (&HF_TIMx, &sClearInputConfig, HF_TIMx_CH1);
  HAL_TIM_ConfigOCrefClear (&HF_TIMx, &sClearInputConfig, HF_TIMx_CH2);
  HAL_TIM_ConfigOCrefClear (&HF_TIMx, &sClearInputConfig, HF_TIMx_CH3);

  // stop TIM during Breakpoint
  __HAL_FREEZE_TIM1_DBGMCU();
  __HAL_TIM_ENABLE_IT (&htim1, TIM_IT_BREAK);

  // REGULAR CHANNELS CONFIGURATION
  // Current feedabck
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = ADC_CH_1;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_CH_1_ST;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel (&hadc1, &sConfig);

  // Bus voltage
  sConfig.Channel = ADC_CH_3;
  sConfig.SamplingTime = ADC_CH_3_ST;
  HAL_ADC_ConfigChannel (&hadc1, &sConfig);

  // Temperature feedback
  sConfig.Channel = ADC_CH_4;
  sConfig.SamplingTime = ADC_CH_4_ST;
  HAL_ADC_ConfigChannel (&hadc1, &sConfig);

  // BEMF feedback phase A
  sConfig.Channel = ADC_Bemf_CH1;
  sConfig.SamplingTime = ADC_Bemf_CH1_ST;
  HAL_ADC_ConfigChannel (&hadc1, &sConfig);

  // BEMF feedback phase B
  sConfig.Channel = ADC_Bemf_CH2;
  sConfig.SamplingTime = ADC_Bemf_CH2_ST;
  HAL_ADC_ConfigChannel (&hadc1, &sConfig);

  // BEMF feedback phase C
  sConfig.Channel = ADC_Bemf_CH3;
  sConfig.SamplingTime = ADC_Bemf_CH3_ST;
  HAL_ADC_ConfigChannel (&hadc1, &sConfig);

  // Potentiometer
  sConfig.Channel = ADC_CH_2;
  sConfig.SamplingTime = ADC_CH_2_ST;
  HAL_ADC_ConfigChannel (&hadc1, &sConfig);
  }
//}}}
//{{{
void MC_ADC_Channel (uint32_t adc_ch) {

  ADCx.Instance->CR |= ADC_CR_ADSTP;
  while (ADCx.Instance->CR & ADC_CR_ADSTP);

  // Clear the old SQx bits for the selected rank
  ADCx.Instance->SQR1 &= ~__HAL_ADC_SQR1_RK (ADC_SQR2_SQ5, 1);

  // Set the SQx bits for the selected rank
  ADCx.Instance->SQR1 |= __HAL_ADC_SQR1_RK (adc_ch, 1);
  ADCx.Instance->CR |= ADC_CR_ADSTART;
  }
//}}}

//{{{
void MC_EnableInput_CH1_E_CH2_E_CH3_D() {
  HAL_GPIO_WritePin (GPIO_PORT_1, GPIO_CH1, GPIO_SET);    //EN1 ENABLE
  HAL_GPIO_WritePin (GPIO_PORT_1, GPIO_CH2, GPIO_SET);    //EN2 DISABLE
  HAL_GPIO_WritePin (GPIO_PORT_1, GPIO_CH3, GPIO_RESET);  //EN3 ENABLE
  }
//}}}
//{{{
void MC_EnableInput_CH1_E_CH2_D_CH3_E() {
  HAL_GPIO_WritePin (GPIO_PORT_1, GPIO_CH1, GPIO_SET);    //EN1 ENABLE
  HAL_GPIO_WritePin (GPIO_PORT_1, GPIO_CH2, GPIO_RESET);  //EN2 DISABLE
  HAL_GPIO_WritePin (GPIO_PORT_1, GPIO_CH3, GPIO_SET);    //EN3 ENABLE
  }
//}}}
//{{{
void MC_EnableInput_CH1_D_CH2_E_CH3_E() {
  HAL_GPIO_WritePin (GPIO_PORT_1, GPIO_CH1, GPIO_RESET);  //EN1 DISABLE
  HAL_GPIO_WritePin (GPIO_PORT_1, GPIO_CH2, GPIO_SET);    //EN2 ENABLE
  HAL_GPIO_WritePin (GPIO_PORT_1, GPIO_CH3, GPIO_SET);    //EN3 ENABLE
  }
//}}}
//{{{
void MC_DisableInput_CH1_D_CH2_D_CH3_D() {
  HAL_GPIO_WritePin (GPIO_PORT_1, GPIO_CH1, GPIO_RESET);  //EN1 DISABLE
  HAL_GPIO_WritePin (GPIO_PORT_1, GPIO_CH2, GPIO_RESET);  //EN2 DISABLE
  HAL_GPIO_WritePin (GPIO_PORT_1, GPIO_CH3, GPIO_RESET);  //EN3 DISABLE
  }
//}}}

//{{{
void MC_Start_PWM_driving() {
  HAL_TIM_PWM_Start (&HF_TIMx, HF_TIMx_CH1); // TIM1_CH1 ENABLE
  HAL_TIM_PWM_Start (&HF_TIMx, HF_TIMx_CH2); // TIM1_CH2 ENABLE
  HAL_TIM_PWM_Start (&HF_TIMx, HF_TIMx_CH3); // TIM1_CH3 ENABLE
  }
//}}}
//{{{
void MC_Stop_PWM_driving() {
  HAL_TIM_PWM_Stop (&HF_TIMx, HF_TIMx_CH1); // TIM1_CH1 DISABLE
  HAL_TIM_PWM_Stop (&HF_TIMx, HF_TIMx_CH2); // TIM1_CH2 DISABLE
  HAL_TIM_PWM_Stop (&HF_TIMx, HF_TIMx_CH3); // TIM1_CH3 DISABLE
  }
//}}}
//{{{
void MC_HF_TIMx_SetDutyCycle_CH1 (uint16_t CCR_value) {
  HF_TIMx.Instance->HF_TIMx_CCR1 = CCR_value;
  }
//}}}
//{{{
void MC_HF_TIMx_SetDutyCycle_CH2 (uint16_t CCR_value) {
  HF_TIMx.Instance->HF_TIMx_CCR2 = CCR_value;
  }
//}}}
//{{{
void MC_HF_TIMx_SetDutyCycle_CH3 (uint16_t CCR_value) {
  HF_TIMx.Instance->HF_TIMx_CCR3 = CCR_value;
  }
//}}}

//{{{
void MC_Current_Reference_Start() {

  REFx.Instance->CCR1 = 0;
  HAL_TIM_PWM_Start (&REFx, HF_TIMx_CH1);
  }
//}}}
//{{{
void MC_Current_Reference_Stop() {

  REFx.Instance->CCR1 = 0;
  HAL_TIM_PWM_Stop (&REFx, HF_TIMx_CH1);
  }
//}}}
//{{{
void MC_Current_Reference_Setvalue (uint16_t Iref) {
  REFx.Instance->CCR1 = (uint32_t)(Iref * REFx.Instance->ARR) / 4096;
  }
//}}}

//{{{
void NUCLEO_LED_ON() {
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  }
//}}}
//{{{
void NUCLEO_LED_OFF() {
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  }
//}}}
