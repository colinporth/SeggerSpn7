// ihm07m1.c
#include "ihm07m1.h"
#include "sixStepLib.h"

#define GPIO_CH1  GPIO_PIN_10
#define GPIO_CH2  GPIO_PIN_11
#define GPIO_CH3  GPIO_PIN_12

#define ADC_CH_1_ST       ADC_SAMPLETIME_1CYCLE_5    // CURRENT sampling time
#define ADC_CH_2_ST       ADC_SAMPLETIME_181CYCLES_5 // SPEED sampling time
#define ADC_CH_3_ST       ADC_SAMPLETIME_181CYCLES_5 // VBUS sampling time
#define ADC_CH_4_ST       ADC_SAMPLETIME_181CYCLES_5 // TEMP sampling time
#define ADC_Bemf_CH1_ST   ADC_SAMPLETIME_61CYCLES_5  // BEMF1 sampling time
#define ADC_Bemf_CH2_ST   ADC_SAMPLETIME_61CYCLES_5  // BEMF2 sampling time
#define ADC_Bemf_CH3_ST   ADC_SAMPLETIME_61CYCLES_5  // BEMF3 sampling time

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;
DAC_HandleTypeDef hdac;

extern SIXSTEP_Base_InitTypeDef sixStep;
extern SIXSTEP_PI_PARAM_InitTypeDef_t PI_parameters;

// irq handlers
extern "C" {
  //{{{
  void SysTick_Handler() {

    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
    }
  //}}}
  //{{{
  void ADC1_IRQHandler() {

    //printf ("ADC1_IRQHandler\n");
    HAL_ADC_IRQHandler (&hadc1);
    }
  //}}}
  //{{{
  void TIM1_BRK_TIM15_IRQHandler() {

    printf ("TIM1_BRK_TIM15_IRQHandler\n");

    if (__HAL_TIM_GET_FLAG (&htim1, TIM_FLAG_BREAK) != RESET) {
      MC_StopMotor();
      sixStep.STATUS = OVERCURRENT;
      }

    HAL_TIM_IRQHandler (&htim1);
    }
  //}}}
  //{{{
  void TIM6_DAC_IRQHandler() {

    //printf ("TIM6_DAC_IRQHandler\n");
    HAL_TIM_IRQHandler (&htim6);
    HAL_DAC_IRQHandler (&hdac);
    }
  //}}}
  //{{{
  void EXTI15_10_IRQHandler() {

    printf ("EXTI15_10_IRQHandler\n");
    HAL_GPIO_EXTI_IRQHandler (GPIO_PIN_13);
    }
  //}}}
  }

//{{{
void _Error_Handler (const char* file, int line) {
  printf ("Error %s %n\n", file, line);
  while(1) {
    }
  }
//}}}

//{{{
void GPIO_Init() {

  // GPIO Ports Clock Enable
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // config GPIO PC13 input interrupt - blueButton
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  // config GPIO PC10 PC11 PC12 output
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);

  // config GPIO pin PB2 - output ihm07m1 red led
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

  // EXTI interrupt init
  HAL_NVIC_SetPriority (EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (EXTI15_10_IRQn);
  }
//}}}
//{{{
void ADC1_Init() {
// ADC1 GPIO Configuration
// PC1 > ADC1_IN7
// PC2 > ADC1_IN8
// PC3 > ADC1_IN9
// PA1 > ADC1_IN2
// PA7 > ADC1_IN15
// PB0 > ADC1_IN11
// PB1 > ADC1_IN12

  __HAL_RCC_ADC1_CLK_ENABLE();

  // config GPIO C adc inputs
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);

  // config GPIO A adc inputs
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  // config GPIO B adc inputs
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

  // ADC1 interrupt Init
  HAL_NVIC_SetPriority (ADC1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (ADC1_IRQn);

  ADC_ChannelConfTypeDef sConfig;
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init (&hadc1) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);

  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel (&hadc1, &sConfig) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);
  }
//}}}
//{{{
void TIM1_Init() {

  __HAL_RCC_TIM1_CLK_ENABLE();

  // config GPIO PA6 TIM1_BKIN timer output
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  // config GPIO PA12 TIM1_ETR timer output
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_TIM1;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 719;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init (&htim1) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);

  TIM_ClockConfigTypeDef sClockSourceConfig;
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource (&htim1, &sClockSourceConfig) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);
  if (HAL_TIM_PWM_Init (&htim1) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);

  TIM_ClearInputConfigTypeDef sClearInputConfig;
  sClearInputConfig.ClearInputState = ENABLE;
  sClearInputConfig.ClearInputSource = TIM_CLEARINPUTSOURCE_ETR;
  sClearInputConfig.ClearInputPolarity = TIM_CLEARINPUTPOLARITY_NONINVERTED;
  sClearInputConfig.ClearInputPrescaler = TIM_CLEARINPUTPRESCALER_DIV1;
  sClearInputConfig.ClearInputFilter = 0;
  if (HAL_TIM_ConfigOCrefClear (&htim1, &sClearInputConfig, TIM_CHANNEL_1) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);
  if (HAL_TIM_ConfigOCrefClear (&htim1, &sClearInputConfig, TIM_CHANNEL_2) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);
  if (HAL_TIM_ConfigOCrefClear (&htim1, &sClearInputConfig, TIM_CHANNEL_3) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);

  TIM_MasterConfigTypeDef sMasterConfig;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization (&htim1, &sMasterConfig) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);

  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 575;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel (&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);
  if (HAL_TIM_PWM_ConfigChannel (&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);
  if (HAL_TIM_PWM_ConfigChannel (&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);

  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_LOW;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime (&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  // config PA8 PA9 PA10 PWM YUW out
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
  }
//}}}
//{{{
void TIM2_Init() {

  __HAL_RCC_TIM2_CLK_ENABLE();

  // config GPIO PB3 PB10 timer outputs
  // PB10 > TIM2_CH3
  // PB3  > TIM2_CH2
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init (&htim2) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  TIM_ClockConfigTypeDef sClockSourceConfig;
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource (&htim2, &sClockSourceConfig) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);
  if (HAL_TIM_PWM_Init (&htim2) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);

  TIM_MasterConfigTypeDef sMasterConfig;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization (&htim2, &sMasterConfig) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel (&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  // config PA5
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
  }
//}}}
//{{{
void TIM6_Init() {

  // config TIM6 interrupt
  __HAL_RCC_TIM6_CLK_ENABLE();
  HAL_NVIC_SetPriority (TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ (TIM6_DAC_IRQn);

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 11;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 24000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init (&htim6) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);

  TIM_MasterConfigTypeDef sMasterConfig;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization (&htim6, &sMasterConfig) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);
  }
//}}}
//{{{
void TIM16_Init() {

  __HAL_RCC_TIM16_CLK_ENABLE();

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1439;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init (&htim16) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);
  if (HAL_TIM_PWM_Init (&htim16) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);

  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 720;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel (&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);

  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime (&htim16, &sBreakDeadTimeConfig) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);

  // config PB4 - current Ref
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM16;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);
  }
//}}}
//{{{
void DAC_Init() {
// DAC GPIO Configuration
// PA4 > DAC_OUT1

  __HAL_RCC_DAC1_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  // DAC interrupt Init
  HAL_NVIC_SetPriority (TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ (TIM6_DAC_IRQn);

  hdac.Instance = DAC;
  if (HAL_DAC_Init (&hdac) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  DAC_ChannelConfTypeDef sConfig;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel (&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);
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
void MC_BemfDelayCalc() {

 if (PI_parameters.Reference >= 0) {
   if (sixStep.speed_fdbk_filtered <= 12000 && sixStep.speed_fdbk_filtered > 10000)
      sixStep.demagn_value = DEMAGN_VAL_1;
    else if (sixStep.speed_fdbk_filtered <= 10000 && sixStep.speed_fdbk_filtered > 7800)
      sixStep.demagn_value = DEMAGN_VAL_2;
    else if (sixStep.speed_fdbk_filtered <= 7800 && sixStep.speed_fdbk_filtered > 6400)
      sixStep.demagn_value = DEMAGN_VAL_3;
    else if (sixStep.speed_fdbk_filtered <= 6400 && sixStep.speed_fdbk_filtered > 5400)
      sixStep.demagn_value = DEMAGN_VAL_4;
    else if (sixStep.speed_fdbk_filtered <= 5400 && sixStep.speed_fdbk_filtered > 4650)
      sixStep.demagn_value = DEMAGN_VAL_5;
    else if (sixStep.speed_fdbk_filtered <= 4650 && sixStep.speed_fdbk_filtered > 4100)
      sixStep.demagn_value = DEMAGN_VAL_6;
    else if (sixStep.speed_fdbk_filtered <= 4100 && sixStep.speed_fdbk_filtered > 3650)
      sixStep.demagn_value = DEMAGN_VAL_7;
    else if (sixStep.speed_fdbk_filtered <= 3650 && sixStep.speed_fdbk_filtered > 3300)
      sixStep.demagn_value = DEMAGN_VAL_8;
    else if (sixStep.speed_fdbk_filtered <= 3300 && sixStep.speed_fdbk_filtered > 2600)
      sixStep.demagn_value = DEMAGN_VAL_9;
    else if (sixStep.speed_fdbk_filtered <= 2600 && sixStep.speed_fdbk_filtered > 1800)
      sixStep.demagn_value = DEMAGN_VAL_10;
    else if (sixStep.speed_fdbk_filtered <= 1800 && sixStep.speed_fdbk_filtered > 1500)
      sixStep.demagn_value = DEMAGN_VAL_11;
    else if (sixStep.speed_fdbk_filtered <= 1500 && sixStep.speed_fdbk_filtered > 1300)
      sixStep.demagn_value = DEMAGN_VAL_12;
    else if (sixStep.speed_fdbk_filtered <= 1300 && sixStep.speed_fdbk_filtered > 1000)
      sixStep.demagn_value = DEMAGN_VAL_13;
    else if (sixStep.speed_fdbk_filtered <= 1000 && sixStep.speed_fdbk_filtered > 500)
      sixStep.demagn_value = DEMAGN_VAL_14;
     }
   else {
    if (sixStep.speed_fdbk_filtered >= -12000 && sixStep.speed_fdbk_filtered < -10000)
      sixStep.demagn_value = DEMAGN_VAL_1;
    else if (sixStep.speed_fdbk_filtered >= -10000 && sixStep.speed_fdbk_filtered < -7800)
      sixStep.demagn_value = DEMAGN_VAL_2;
    else if (sixStep.speed_fdbk_filtered >= -7800 && sixStep.speed_fdbk_filtered < -6400)
      sixStep.demagn_value = DEMAGN_VAL_3;
    else if (sixStep.speed_fdbk_filtered >= -6400 && sixStep.speed_fdbk_filtered < -5400)
      sixStep.demagn_value = DEMAGN_VAL_4;
    else if (sixStep.speed_fdbk_filtered >= -5400 && sixStep.speed_fdbk_filtered < -4650)
      sixStep.demagn_value = DEMAGN_VAL_5;
    else if (sixStep.speed_fdbk_filtered >= -4650 && sixStep.speed_fdbk_filtered < -4100)
      sixStep.demagn_value = DEMAGN_VAL_6;
    else if (sixStep.speed_fdbk_filtered >= -4100 && sixStep.speed_fdbk_filtered < -3650)
      sixStep.demagn_value = DEMAGN_VAL_7;
    else if (sixStep.speed_fdbk_filtered >= -3650 && sixStep.speed_fdbk_filtered < -3300)
      sixStep.demagn_value = DEMAGN_VAL_8;
    else if (sixStep.speed_fdbk_filtered >= -3300 && sixStep.speed_fdbk_filtered < -2650)
      sixStep.demagn_value = DEMAGN_VAL_9;
    else if (sixStep.speed_fdbk_filtered >= -2600 && sixStep.speed_fdbk_filtered <-1800)
      sixStep.demagn_value = DEMAGN_VAL_10;
    else if (sixStep.speed_fdbk_filtered >= -1800 && sixStep.speed_fdbk_filtered < -1500)
      sixStep.demagn_value = DEMAGN_VAL_11;
    else if (sixStep.speed_fdbk_filtered >= -1500 && sixStep.speed_fdbk_filtered < -1300)
      sixStep.demagn_value = DEMAGN_VAL_12;
    else if (sixStep.speed_fdbk_filtered >= -1300 && sixStep.speed_fdbk_filtered < -1000)
      sixStep.demagn_value = DEMAGN_VAL_13;
    else if (sixStep.speed_fdbk_filtered >= -1000 && sixStep.speed_fdbk_filtered < -500)
      sixStep.demagn_value = DEMAGN_VAL_14;
    }
  }
//}}}

//{{{
void MC_EnableInput_CH1_E_CH2_E_CH3_D() {
  HAL_GPIO_WritePin (GPIOC, GPIO_CH1, GPIO_PIN_SET);    // EN1 ENABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_CH2, GPIO_PIN_SET);    // EN2 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_CH3, GPIO_PIN_RESET);  // EN3 ENABLE
  }
//}}}
//{{{
void MC_EnableInput_CH1_E_CH2_D_CH3_E() {
  HAL_GPIO_WritePin (GPIOC, GPIO_CH1, GPIO_PIN_SET);   // EN1 ENABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_CH2, GPIO_PIN_RESET); // EN2 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_CH3, GPIO_PIN_SET);   // EN3 ENABLE
  }
//}}}
//{{{
void MC_EnableInput_CH1_D_CH2_E_CH3_E() {
  HAL_GPIO_WritePin (GPIOC, GPIO_CH1, GPIO_PIN_RESET); // EN1 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_CH2, GPIO_PIN_SET);   // EN2 ENABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_CH3, GPIO_PIN_SET);   // EN3 ENABLE
  }
//}}}
//{{{
void MC_DisableInput_CH1_D_CH2_D_CH3_D() {
  HAL_GPIO_WritePin (GPIOC, GPIO_CH1, GPIO_PIN_RESET);  // EN1 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_CH2, GPIO_PIN_RESET);  // EN2 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_CH3, GPIO_PIN_RESET);  // EN3 DISABLE
  }
//}}}

//{{{
void MC_Start_PWM() {
  HAL_TIM_PWM_Start (&HF_TIMx, HF_TIMx_CH1); // TIM1_CH1 ENABLE
  HAL_TIM_PWM_Start (&HF_TIMx, HF_TIMx_CH2); // TIM1_CH2 ENABLE
  HAL_TIM_PWM_Start (&HF_TIMx, HF_TIMx_CH3); // TIM1_CH3 ENABLE
  }
//}}}
//{{{
void MC_Stop_PWM() {
  HAL_TIM_PWM_Stop (&HF_TIMx, HF_TIMx_CH1); // TIM1_CH1 DISABLE
  HAL_TIM_PWM_Stop (&HF_TIMx, HF_TIMx_CH2); // TIM1_CH2 DISABLE
  HAL_TIM_PWM_Stop (&HF_TIMx, HF_TIMx_CH3); // TIM1_CH3 DISABLE
  }
//}}}
//{{{
void MC_HF_TIMx_SetDutyCycle_CH1 (uint16_t value) {
  HF_TIMx.Instance->HF_TIMx_CCR1 = value;
  }
//}}}
//{{{
void MC_HF_TIMx_SetDutyCycle_CH2 (uint16_t value) {
  HF_TIMx.Instance->HF_TIMx_CCR2 = value;
  }
//}}}
//{{{
void MC_HF_TIMx_SetDutyCycle_CH3 (uint16_t value) {
  HF_TIMx.Instance->HF_TIMx_CCR3 = value;
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
void MC_Current_Reference_Setvalue (uint16_t value) {
  REFx.Instance->CCR1 = (uint32_t)(value * REFx.Instance->ARR) / 4096;
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

//{{{
void MC_Nucleo_Init() {

  GPIO_Init();
  ADC1_Init();
  TIM1_Init();
  TIM2_Init();
  TIM6_Init();
  TIM16_Init();
  DAC_Init();

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
