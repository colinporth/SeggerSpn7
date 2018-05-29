// main.c
//{{{  F303RE pins
// CN7           1 PC10 ->Enable_CH1-L6230    Enable_CH2-L6230<- PC11 2
//               3 PC12 ->Enable_CH3-L6230                       PD2  4
//               5 VDD                                           E5V  6
//               7 BOOT0                                         GND  8
//               9                                                   10
//              11                                             IOREF 12
//              13 PA13                                          RST 14
//              15 PA14                                         +3.3 16
//              17 PA15 <-Encoder A/Hall H1        Encoder/Hall<- +5 18
//              19 GND                                           GND 20
//              21 PB7                                           GND 22
//              23 PC13 <-BlueButton                             VIN 24
//              25 PC14                                              26
//              27 PC15                          Curr_fdbk_PhA-> PA0 28
//              29 PF0                            VBUS_sensing-> PA1 30 ->ADC1_IN2
//              31 PF1                                  DAC_Ch<- PA4 32
//              33 VBAT                          BEMF2_sensing-> PB0 34 ->ADC1_IN11
//   ADC1_IN8<- 35 PC2 <-Temperature feedback    Curr_fdbk_PhB-> PC1 36 ->ADC1_IN7
//   ADC1_IN9<- 37 PC3 <-BEMF1_sensing           Curr_fdbk_PhC-> PC0 38
//
// CN10          1 PC9                                           PC8  2
//               3 PB8                                           PC6  4
//               5 PB9                                           PC5  6
//               7 AVDD                                          U5V  8
//               9 GND                                               10
//              11 PA5 GPIO/DAC/PWM                       CPOUT PA12 12 ->TIM1_ETR
//       TIM1<- 13 PA6 DIAG/ENABLE/BKIN1      DIAG/ENABLE/BKIN2 PA11 14
//  ADC1_IN15<- 15 PA7 <-BEMF3_sensing                          PB12 16
//              17 PB6                                          PB11 18
//              19 PC7                                           GND 20
//              21 PA9 ->VH_PWM                          LedRed<-PB2 22
//              23 PA8 ->UH_PWM                   POTENTIOMETER->PB1 24 ->ADC1_IN12
//       TIM2<- 25 PB10 <-Encoder Z/Hall H3      BEMF3_sensing->PB15 26
//              27 PB4 CurrentRef             DIAG/ENABLE/BKIN1 PB14 28
//              29 PB5 GPIO/DAC/PWM                GPIO/DAC/PWM PB13 30
//       TIM2<- 31 PB3 <-Encoder B/Hall H2                      AGND 32
//              33 PA10 ->WH_PWM                                 PC4 34
//              35 PA2                                               36
//              37 PA3                                               38
//}}}
//{{{  includes
#include "stm32f3xx_nucleo.h"

#include "ihm07m1.h"
#include "sixStepLib.h"
//}}}
//{{{  vars
ADC_HandleTypeDef hAdc1;
ADC_HandleTypeDef hAdc2;
ADC_HandleTypeDef hAdc3;
TIM_HandleTypeDef hTim1;
TIM_HandleTypeDef hTim6;
TIM_HandleTypeDef hTim16;

cSixStep sixStep;
cPiParam piParam;

uint32_t mLastButtonPress = 0;
uint16_t mAlignTicks = 1;
uint16_t mMotorStartupCount = 1;

uint16_t Rotor_poles_pairs;               //  Number of pole pairs of the motor
uint32_t mech_accel_hz = 0;               //  Hz -- Mechanical acceleration rate
uint32_t constant_k = 0;                  //  1/3*mech_accel_hz

uint32_t Time_vector_tmp = 0;             //  Startup variable
uint32_t Time_vector_prev_tmp = 0 ;       //  Startup variable
uint32_t T_single_step = 0;               //  Startup variable
uint32_t T_single_step_first_value = 0;   //  Startup variable

int32_t delta = 0;                        //  Startup variable
uint16_t index_array = 1;                 //  Speed filter variable
int16_t speed_tmp_array[FILTER_DEEP];     //  Speed filter variable
uint16_t speed_tmp_buffer[FILTER_DEEP];   //  Potentiometer filter variable
bool array_completed = false;             //  Speed filter variable
bool buffer_completed = false;            //  Potentiometer filter variable

uint16_t HFBufferIndex = 0;               //  High-Frequency Buffer Index
uint16_t HFBuffer[HFBUFFERSIZE];          //  Buffer for Potentiometer Value Filtering at the High-Frequency ADC conversion

uint32_t ARR_LF = 0;                      //  Autoreload LF TIM variable
int32_t Mech_Speed_RPM = 0;               //  Mechanical motor speed
int32_t El_Speed_Hz = 0;                  //  Electrical motor speed

uint16_t target_speed = TARGET_SPEED > 0 ? TARGET_SPEED : -TARGET_SPEED;

uint16_t index_ARR_step = 1;
uint32_t n_zcr_startup = 0;
uint16_t shift_n_sqrt = 14;

uint16_t cnt_bemf_event = 0;
bool startup_bemf_failure = false;
bool speed_fdbk_error = false;

int32_t speed_sum_sp_filt = 0;
int32_t speed_sum_pot_filt = 0;
uint16_t index_pot_filt = 1;
int16_t potent_filtered = 0;

uint32_t counter_ARR_Bemf = 0;
uint64_t constant_multiplier_tmp = 0;
//}}}
//{{{
extern "C" {
  void ADC1_IRQHandler() { HAL_ADC_IRQHandler (&hAdc1); }
  void ADC2_IRQHandler() { HAL_ADC_IRQHandler (&hAdc2); }
  void ADC3_IRQHandler() { HAL_ADC_IRQHandler (&hAdc3); }

  void TIM6_DAC_IRQHandler() { HAL_TIM_IRQHandler (&hTim6); }
  void EXTI15_10_IRQHandler() { HAL_GPIO_EXTI_IRQHandler (GPIO_PIN_13); }
  //{{{
  void TIM1_BRK_TIM15_IRQHandler() {

    if (__HAL_TIM_GET_FLAG (&hTim1, TIM_FLAG_BREAK) != RESET)
      mcPanic();

    HAL_TIM_IRQHandler (&hTim1);
    }
  //}}}
  //{{{
  void SysTick_Handler() {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
    }
  //}}}
  }
//}}}

//{{{  ihm07m1
//{{{
void GPIO_Init() {
// config
// PB2  -> redLed
// PC13 <- blueButton + extInt

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // config gpio pin PB2 - output ihm07m1 red led
  GPIO_InitTypeDef gpioInit;
  gpioInit.Pin = GPIO_PIN_2;
  gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
  gpioInit.Pull = GPIO_NOPULL;
  gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOB, &gpioInit);
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  // config gpio PC13 input interrupt - blueButton
  gpioInit.Pin = GPIO_PIN_13;
  gpioInit.Mode = GPIO_MODE_IT_RISING;
  gpioInit.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOC, &gpioInit);

  // EXTI interrupt init
  HAL_NVIC_SetPriority (EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (EXTI15_10_IRQn);
  }
//}}}
//{{{
void ADC_Init() {
//  PC1 -> ADC12_IN7  curr_fdbk2 - 1shunt
//  PB1 -> ADC3_IN1   pot
//  PA1 -> ADC1_IN2   vbus
//  PC2 -> ADC12_IN8  temp

//  PC3 -> ADC12_IN9  bemf 1/A
//  PB0 -> ADC3_IN12  bemf 2/B
//  PA7 -> ADC2_IN4   bemf 3/C

// unused
//  PA0 -> ADC1_IN1   curr_fdbk1 - 3shunt
//  PC0 -> ADC12_IN6  curr_fdbk3 - 3shunt
//  PA15-> TIM2_CH1   A/H1
//  PB3 -> TIM2_CH2   B/H2
//  PA10-> TIM2_CH4   C/H3

  //{{{  config clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_ADC2_CLK_ENABLE();
  __HAL_RCC_ADC34_CLK_ENABLE();
  //}}}
  //{{{  config PA1 PA7 adc input pin
  GPIO_InitTypeDef gpioInit;
  gpioInit.Pin = GPIO_PIN_1 | GPIO_PIN_7;
  gpioInit.Mode = GPIO_MODE_ANALOG;
  gpioInit.Pull = GPIO_NOPULL;

  HAL_GPIO_Init (GPIOA, &gpioInit);
  //}}}
  //{{{  config PB0 PB0 adc input pin
  gpioInit.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  HAL_GPIO_Init (GPIOB, &gpioInit);
  //}}}
  //{{{  config PC1 PC2 PC3 adc input pin
  gpioInit.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  HAL_GPIO_Init (GPIOC, &gpioInit);
  //}}}

  //{{{  init adc1
  hAdc1.Instance = ADC1;

  hAdc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hAdc1.Init.Resolution = ADC_RESOLUTION_12B;

  hAdc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hAdc1.Init.ContinuousConvMode = DISABLE;
  hAdc1.Init.DiscontinuousConvMode = DISABLE;

  hAdc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hAdc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hAdc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hAdc1.Init.NbrOfConversion = 1;

  hAdc1.Init.DMAContinuousRequests = DISABLE;
  hAdc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hAdc1.Init.LowPowerAutoWait = DISABLE;
  hAdc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

  if (HAL_ADC_Init (&hAdc1) != HAL_OK)
    printf ("HAL_ADC_Init failed\n");
  //}}}
  //{{{  init channelConfig
  ADC_ChannelConfTypeDef channelConfig;
  channelConfig.Rank = 1;
  channelConfig.SingleDiff = ADC_SINGLE_ENDED;
  channelConfig.OffsetNumber = ADC_OFFSET_NONE;
  channelConfig.Offset = 0;
  //}}}
  //{{{  config current feedback - channel 7
  channelConfig.Channel = ADC_CHANNEL_7;
  channelConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel (&hAdc1, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");
  //}}}
  //{{{  config Vbus - channel 2
  channelConfig.Channel = ADC_CHANNEL_2;
  channelConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;

  if (HAL_ADC_ConfigChannel (&hAdc1, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");
  //}}}
  //{{{  config temperature - channel 8
  channelConfig.Channel = ADC_CHANNEL_8;
  channelConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;

  if (HAL_ADC_ConfigChannel (&hAdc1, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");
  //}}}
  //{{{  config bemf feedback phase 1/A - channel 9
  channelConfig.Channel = ADC_CHANNEL_9;
  channelConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;

  if (HAL_ADC_ConfigChannel (&hAdc1, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");
  //}}}

  //{{{  init adc2
  hAdc2.Instance = ADC2;

  hAdc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hAdc2.Init.Resolution = ADC_RESOLUTION_12B;

  hAdc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hAdc2.Init.ContinuousConvMode = DISABLE;
  hAdc2.Init.DiscontinuousConvMode = DISABLE;

  hAdc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hAdc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;

  hAdc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hAdc2.Init.NbrOfConversion = 1;
  hAdc2.Init.DMAContinuousRequests = DISABLE;
  hAdc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

  hAdc2.Init.LowPowerAutoWait = DISABLE;
  hAdc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

  if (HAL_ADC_Init (&hAdc2) != HAL_OK)
    printf ("HAL_ADC_Init failed\n");
  //}}}
  //{{{  config bemf feedback phase 3/C - channel 4
  channelConfig.Channel = ADC_CHANNEL_4;
  channelConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;

  if (HAL_ADC_ConfigChannel (&hAdc2, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");
  //}}}

  //{{{  init adc3
  hAdc3.Instance = ADC3;

  hAdc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hAdc3.Init.Resolution = ADC_RESOLUTION_12B;

  hAdc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hAdc3.Init.ContinuousConvMode = DISABLE;
  hAdc3.Init.DiscontinuousConvMode = DISABLE;

  hAdc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hAdc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;

  hAdc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hAdc3.Init.NbrOfConversion = 1;
  hAdc3.Init.DMAContinuousRequests = DISABLE;
  hAdc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

  hAdc3.Init.LowPowerAutoWait = DISABLE;
  hAdc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

  if (HAL_ADC_Init (&hAdc3) != HAL_OK)
    printf ("HAL_ADC_Init failed\n");
  //}}}
  //{{{  config potentiometer - channel 1
  channelConfig.Channel = ADC_CHANNEL_1;
  channelConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;

  if (HAL_ADC_ConfigChannel (&hAdc3, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");
  //}}}
  //{{{  config bemf feedback phase 2/B - channel 12
  channelConfig.Channel = ADC_CHANNEL_12;
  channelConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;

  if (HAL_ADC_ConfigChannel (&hAdc3, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");
  //}}}

  // ADC1,2,3 interrupts
  HAL_NVIC_SetPriority (ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (ADC1_2_IRQn);
  HAL_NVIC_SetPriority (ADC3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (ADC3_IRQn);
  }
//}}}
//{{{
void TIM1_Init() {
// 3phase PWM timer

  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  //{{{  config PC10 PC11 PC12 YUW enable output, disable
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);
  //}}}
  //{{{  config PA6 TIM1_BKIN timer output
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
  //}}}
  //{{{  config PA12 TIM1_ETR timer output
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Alternate = GPIO_AF11_TIM1;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
  //}}}

  //{{{  TIM1 init
  hTim1.Instance = TIM1;
  hTim1.Init.Prescaler = 0;
  hTim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  hTim1.Init.Period = 719;
  hTim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  hTim1.Init.RepetitionCounter = 0;
  hTim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init (&hTim1) != HAL_OK)
    printf ("HAL_TIM_Base_Init failed\n");
  //}}}
  //{{{  TIM1 clockSource
  TIM_ClockConfigTypeDef clockSourceConfig;
  clockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource (&hTim1, &clockSourceConfig) != HAL_OK)
    printf ("HAL_TIM_ConfigClockSource failed\n");
  //}}}
  if (HAL_TIM_PWM_Init (&hTim1) != HAL_OK)
    printf ("HAL_TIM_PWM_Init failed\n");

  //{{{  TIM1 config ETR clearInput
  TIM_ClearInputConfigTypeDef clearInputConfig;

  clearInputConfig.ClearInputState = ENABLE;
  clearInputConfig.ClearInputSource = TIM_CLEARINPUTSOURCE_ETR;
  clearInputConfig.ClearInputPolarity = TIM_CLEARINPUTPOLARITY_NONINVERTED;
  clearInputConfig.ClearInputPrescaler = TIM_CLEARINPUTPRESCALER_DIV1;
  clearInputConfig.ClearInputFilter = 0;

  if (HAL_TIM_ConfigOCrefClear (&hTim1, &clearInputConfig, TIM_CHANNEL_1) != HAL_OK)
    printf ("HAL_TIM_ConfigOCrefClear 1 failed\n");
  if (HAL_TIM_ConfigOCrefClear (&hTim1, &clearInputConfig, TIM_CHANNEL_2) != HAL_OK)
    printf ("HAL_TIM_ConfigOCrefClear 2 failed\n");
  if (HAL_TIM_ConfigOCrefClear (&hTim1, &clearInputConfig, TIM_CHANNEL_3) != HAL_OK)
    printf ("HAL_TIM_ConfigOCrefClear 3 failed\n");
  //}}}
  //{{{  TIM1 config master synchronisation
  TIM_MasterConfigTypeDef masterConfig;
  masterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  masterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  masterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;

  if (HAL_TIMEx_MasterConfigSynchronization (&hTim1, &masterConfig) != HAL_OK)
    printf ("HAL_TIMEx_MasterConfigSynchronization failed\n");
  //}}}
  //{{{  TIM1 config OC CH1,CH2,CH3
  TIM_OC_InitTypeDef configOC;

  configOC.OCMode = TIM_OCMODE_PWM1;
  configOC.Pulse = 575;
  configOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  configOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  configOC.OCFastMode = TIM_OCFAST_DISABLE;
  configOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  configOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel (&hTim1, &configOC, TIM_CHANNEL_1) != HAL_OK)
    printf ("HAL_TIM_PWM_ConfigChannel1 failed\n");
  if (HAL_TIM_PWM_ConfigChannel (&hTim1, &configOC, TIM_CHANNEL_2) != HAL_OK)
    printf ("HAL_TIM_PWM_ConfigChannel2 failed\n");
  if (HAL_TIM_PWM_ConfigChannel (&hTim1, &configOC, TIM_CHANNEL_3) != HAL_OK)
    printf ("HAL_TIM_PWM_ConfigChannel3 failed\n");
  //}}}
  //{{{  TIM1 config breakDeadTime
  TIM_BreakDeadTimeConfigTypeDef breakDeadTimeConfig;
  breakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  breakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  breakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  breakDeadTimeConfig.DeadTime = 0;
  breakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  breakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  breakDeadTimeConfig.BreakFilter = 0;
  breakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  breakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_LOW;
  breakDeadTimeConfig.Break2Filter = 0;
  breakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

  if (HAL_TIMEx_ConfigBreakDeadTime (&hTim1, &breakDeadTimeConfig) != HAL_OK)
    printf ("HAL_TIMEx_ConfigBreakDeadTime failed\n");
  //}}}
  //{{{  config PA8 PA9 PA10 YUW pwm output
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
  //}}}

  // stop TIM during Breakpoint
  __HAL_FREEZE_TIM1_DBGMCU();

  // enable break interrupt
  __HAL_TIM_ENABLE_IT (&hTim1, TIM_IT_BREAK);
  }
//}}}
//{{{
void TIM6_Init() {
// enable lowFreq timer tick interrupt

  // config TIM6 interrupt
  __HAL_RCC_TIM6_CLK_ENABLE();

  hTim6.Instance = TIM6;
  hTim6.Init.Prescaler = 11;
  hTim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  hTim6.Init.Period = 24000;
  hTim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init (&hTim6) != HAL_OK)
    printf ("HAL_TIM_Base_Init failed\n");

  TIM_MasterConfigTypeDef masterConfig;
  masterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  masterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization (&hTim6, &masterConfig) != HAL_OK)
    printf ("HAL_TIMEx_MasterConfigSynchronization failed\n");

  HAL_NVIC_SetPriority (TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ (TIM6_DAC_IRQn);
  }
//}}}
//{{{
void TIM16_Init() {
// currentRef PWM timer
// PB4 -> currentRef

  __HAL_RCC_TIM16_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  hTim16.Instance = TIM16;
  hTim16.Init.Prescaler = 0;
  hTim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  hTim16.Init.Period = 1439;
  hTim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  hTim16.Init.RepetitionCounter = 0;
  hTim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init (&hTim16) != HAL_OK)
    printf ("HAL_TIM_Base_Init failed\n");
  if (HAL_TIM_PWM_Init (&hTim16) != HAL_OK)
    printf ("HAL_TIM_PWM_Init failed\n");

  TIM_OC_InitTypeDef configOC;
  configOC.OCMode = TIM_OCMODE_PWM1;
  configOC.Pulse = 720;
  configOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  configOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  configOC.OCFastMode = TIM_OCFAST_ENABLE;
  configOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  configOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel (&hTim16, &configOC, TIM_CHANNEL_1) != HAL_OK)
    printf ("HAL_TIM_PWM_ConfigChannel failed\n");

  TIM_BreakDeadTimeConfigTypeDef breakDeadTimeConfig;
  breakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  breakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  breakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  breakDeadTimeConfig.DeadTime = 0;
  breakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  breakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  breakDeadTimeConfig.BreakFilter = 0;
  breakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime (&hTim16, &breakDeadTimeConfig) != HAL_OK)
    printf ("HAL_TIMEx_ConfigBreakDeadTime failed\n");

  GPIO_InitTypeDef gpioInit;
  gpioInit.Pin = GPIO_PIN_4;
  gpioInit.Mode = GPIO_MODE_AF_PP;
  gpioInit.Pull = GPIO_NOPULL;
  gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
  gpioInit.Alternate = GPIO_AF1_TIM16;
  HAL_GPIO_Init (GPIOB, &gpioInit);
  }
//}}}

//{{{
void mcNucleoDisableChan() {

  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);  // EN1 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);  // EN2 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);  // EN3 DISABLE
  }
//}}}
//{{{
void mcNucleoEnableInputChan12() {

  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_10, GPIO_PIN_SET);    // EN1 ENABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_11, GPIO_PIN_SET);    // EN2 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);  // EN3 ENABLE
  }
//}}}
//{{{
void mcNucleoEnableInputChan13() {

  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_10, GPIO_PIN_SET);   // EN1 ENABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // EN2 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_12, GPIO_PIN_SET);   // EN3 ENABLE
  }
//}}}
//{{{
void mcNucleoEnableInputChan23() {

  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // EN1 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_11, GPIO_PIN_SET);   // EN2 ENABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_12, GPIO_PIN_SET);   // EN3 ENABLE
  }
//}}}
//{{{
void mcNucleoSetChanCCR (uint16_t value1, uint16_t value2, uint16_t value3) {
  hTim1.Instance->CCR1 = value1;
  hTim1.Instance->CCR2 = value2;
  hTim1.Instance->CCR3 = value3;
  }
//}}}
//{{{
void mcNucleoStartPwm() {
  HAL_TIM_PWM_Start (&hTim1, TIM_CHANNEL_1);  // TIM1_CH1 ENABLE
  HAL_TIM_PWM_Start (&hTim1, TIM_CHANNEL_2);  // TIM1_CH2 ENABLE
  HAL_TIM_PWM_Start (&hTim1, TIM_CHANNEL_3);  // TIM1_CH3 ENABLE
  }
//}}}
//{{{
void mcNucleoStopPwm() {
  HAL_TIM_PWM_Stop (&hTim1, TIM_CHANNEL_1);  // TIM1_CH1 DISABLE
  HAL_TIM_PWM_Stop (&hTim1, TIM_CHANNEL_2);  // TIM1_CH2 DISABLE
  HAL_TIM_PWM_Stop (&hTim1, TIM_CHANNEL_3);  // TIM1_CH3 DISABLE
  }
//}}}

//{{{
void mcNucleoCurrentRefStart() {

  hTim16.Instance->CCR1 = 0;
  HAL_TIM_PWM_Start (&hTim16, TIM_CHANNEL_1);
  }
//}}}
//{{{
void mcNucleoCurrentRefStop() {

  hTim16.Instance->CCR1 = 0;
  HAL_TIM_PWM_Stop (&hTim16, TIM_CHANNEL_1);
  }
//}}}
//{{{
void mcNucleoCurrentRefSetValue (uint16_t value) {

  //printf ("mcCurrent_Reference_Setvalue %d\n", value);
  hTim16.Instance->CCR1 = (uint32_t)(value * hTim16.Instance->ARR) / 4096;
  }
//}}}

//{{{
void mcNucleoAdcChan (ADC_HandleTypeDef* hAdc, uint32_t chan) {

  // stop and wait
  hAdc->Instance->CR |= ADC_CR_ADSTP;
  while (hAdc->Instance->CR & ADC_CR_ADSTP);

  // clear old SQx bits, set SQx bits for the selected chan
  hAdc->Instance->SQR1 &= ~__HAL_ADC_SQR1_RK (ADC_SQR2_SQ5, 1);
  hAdc->Instance->SQR1 |= __HAL_ADC_SQR1_RK (chan, 1);
  hAdc->Instance->CR |= ADC_CR_ADSTART;
  }
//}}}

//{{{
void mcNucleoLedOn() {
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  }
//}}}
//{{{
void mcNucleoLedOff() {
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  }
//}}}

//{{{
void mcNucleoInit() {

  GPIO_Init();
  ADC_Init();
  TIM1_Init();
  TIM6_Init();
  TIM16_Init();
  }
//}}}
//}}}

//{{{
uint64_t fastSqrt (uint64_t wInput) {

  uint64_t tempRoot;
  if (wInput <= (uint64_t)((uint64_t)2097152 << shift_n_sqrt))
    tempRoot = (uint64_t)((uint64_t)128 << shift_n_sqrt);
  else
    tempRoot = (uint64_t)((uint64_t)8192 << shift_n_sqrt);

  uint8_t biter = 0u;
  uint64_t tempRootNew;
  do {
    tempRootNew = (tempRoot + wInput / tempRoot) >> 1;
    if (tempRootNew == tempRoot)
      biter = shift_n_sqrt -1 ;
    else {
      biter ++;
      tempRoot = tempRootNew;
      }
    } while (biter < (shift_n_sqrt - 1));

  return tempRootNew;
  }
//}}}
//{{{
void potSpeedTarget() {

  target_speed = sixStep.mAdcBuffer[1] * MAX_POT_SPEED / 4096;

  if (target_speed < MIN_POT_SPEED)
    target_speed = MIN_POT_SPEED;

  if (target_speed > (MAX_POT_SPEED / VAL_POT_SPEED_DIV))
    target_speed = (MAX_POT_SPEED/VAL_POT_SPEED_DIV);

  //printf ("potSpeedTarget %d\n", target_speed);
  }
//}}}
//{{{
void potSpeed() {

  uint32_t sum = 0;
  uint16_t max = 0;
  for (uint16_t i = 0; i < HFBUFFERSIZE; i++) {
    uint16_t val = HFBuffer[i];
    sum += val;
    if (val > max)
      max = val;
    }

  sum -= max;
  uint16_t potMean = sum / (HFBUFFERSIZE - 1);

  if (!buffer_completed) {
    speed_tmp_buffer[index_pot_filt] = potMean;
    speed_sum_pot_filt = 0;
    for (uint16_t i = 1; i <= index_pot_filt;i++)
      speed_sum_pot_filt = speed_sum_pot_filt + speed_tmp_buffer[i];
    potent_filtered = speed_sum_pot_filt / index_pot_filt;
    index_pot_filt++;

    if (index_pot_filt >= FILTER_DEEP) {
      index_pot_filt = 1;
      buffer_completed = true;
      }
    }

  else {
     index_pot_filt++;
     if (index_pot_filt >= FILTER_DEEP)
       index_pot_filt = 1;

     speed_sum_pot_filt = 0;
     speed_tmp_buffer[index_pot_filt] = potMean;
     uint16_t speed_max = 0;
     for (uint16_t i = 1; i < FILTER_DEEP;i++) {
       uint16_t val = speed_tmp_buffer[i];
       if (val > speed_max)
         speed_max = val;
       speed_sum_pot_filt += val;
       }
     speed_sum_pot_filt -= speed_max;
     potent_filtered = speed_sum_pot_filt / (FILTER_DEEP-2);
    }

  if (potent_filtered==0)
    potent_filtered = 1;

  sixStep.Speed_Ref_filtered = potent_filtered;

  //printf ("potSpeed %d\n", potent_filtered);
  }
//}}}
//{{{
void speedFilter() {

  if (!array_completed) {
    speed_tmp_array[index_array] = sixStep.speed_fdbk;
    speed_sum_sp_filt = 0;
    for (uint16_t i = 1; i <= index_array;i++)
      speed_sum_sp_filt = speed_sum_sp_filt + speed_tmp_array[i];
    sixStep.speed_fdbk_filtered = speed_sum_sp_filt / index_array;
    index_array++;

    if (index_array >= FILTER_DEEP) {
     index_array = 1;
     array_completed = true;
     }
    }

  else {
    index_array++;
    if (index_array >= FILTER_DEEP)
      index_array = 1;

    speed_sum_sp_filt = 0;
    speed_tmp_array[index_array] = sixStep.speed_fdbk;
    for (uint16_t i = 1; i < FILTER_DEEP;i++)
      speed_sum_sp_filt = speed_sum_sp_filt + speed_tmp_array[i];
    sixStep.speed_fdbk_filtered = speed_sum_sp_filt / (FILTER_DEEP-1);
    }
  }
//}}}

//{{{
void sixStepTable (uint8_t step_number) {

  switch (step_number) {
    case 1:
      mcNucleoSetChanCCR (sixStep.pulse_value, 0, 0);
      mcNucleoEnableInputChan12();
      sixStep.mBemfIndex = 2;
      break;

    case 2:
      mcNucleoSetChanCCR (sixStep.pulse_value, 0, 0);
      mcNucleoEnableInputChan13();
      sixStep.mBemfIndex = 1;
      break;

    case 3:
      mcNucleoSetChanCCR (0, sixStep.pulse_value, 0);
      mcNucleoEnableInputChan23();
      sixStep.mBemfIndex = 0;
      break;

    case 4:
      mcNucleoSetChanCCR (0, sixStep.pulse_value, 0);
      mcNucleoEnableInputChan12();
      sixStep.mBemfIndex = 2;
     break;

    case 5:
      mcNucleoSetChanCCR (0, 0, sixStep.pulse_value);
      mcNucleoEnableInputChan13();
      sixStep.mBemfIndex = 1;
      break;

    case 6:
      mcNucleoSetChanCCR (0, 0, sixStep.pulse_value);
      mcNucleoEnableInputChan23();
      sixStep.mBemfIndex = 0;
      break;
     }
  }
//}}}
//{{{
void rampMotor() {

  uint32_t constant_multiplier = 100;
  uint32_t constant_multiplier_2 = 4000000000;

  if (mMotorStartupCount == 1) {
    mech_accel_hz = sixStep.ACCEL * Rotor_poles_pairs / 60;
    constant_multiplier_tmp = (uint64_t)constant_multiplier * (uint64_t)constant_multiplier_2;
    constant_k = constant_multiplier_tmp / (3 * mech_accel_hz);
    mcNucleoCurrentRefSetValue (sixStep.Ireference);
    Time_vector_prev_tmp = 0;
    }

  if (mMotorStartupCount < NUMBER_OF_STEPS) {
    Time_vector_tmp = ((uint64_t)1000 * (uint64_t)1000 * (uint64_t) fastSqrt(((uint64_t)mMotorStartupCount * (uint64_t)constant_k)))/632455;
    delta = Time_vector_tmp - Time_vector_prev_tmp;
    if (mMotorStartupCount == 1) {
      T_single_step_first_value = (2 * 3141) * delta / 1000;
      sixStep.ARR_value = (uint32_t)(65535);
      }
    else {
      T_single_step = (2 * 3141)* delta / 1000;
      sixStep.ARR_value = (uint32_t)(65535 * T_single_step) / T_single_step_first_value;
      }
    }
  else
    mMotorStartupCount = 1;

  if (mMotorStartupCount == 1)
    sixStep.prescaler_value = (((sixStep.SYSCLK_frequency / 1000000) * T_single_step_first_value) / 65535) - 1;

  if ((sixStep.STATUS != ALIGNMENT) && (sixStep.STATUS != START))
    mMotorStartupCount++;
  else
    Time_vector_tmp = 0;

  Time_vector_prev_tmp = Time_vector_tmp;
  }
//}}}
//{{{
void arrStep() {

  if (!sixStep.mAligning)
    sixStep.mAligning = true;

  if (sixStep.mAligned) {
    if (sixStep.VALIDATION_OK) {
      sixStep.ARR_OK = true;
      mMotorStartupCount = 1;
      index_ARR_step = 1;
      }
    else {
      sixStep.STATUS = STARTUP;
      rampMotor();
      if (index_ARR_step < sixStep.numberofitemArr) {
        hTim6.Init.Period = sixStep.ARR_value;
        hTim6.Instance->ARR = (uint32_t)hTim6.Init.Period;
        index_ARR_step++;
        }
      else if (!sixStep.ARR_OK) {
        index_ARR_step = 1;
        sixStep.ACCEL >>= 1;
        if (sixStep.ACCEL < MINIMUM_ACC)
          sixStep.ACCEL = MINIMUM_ACC;
        printf ("STARTUP_FAILURE\n");
        mcStopMotor();
        sixStep.STATUS = STARTUP_FAILURE;
        }
      }
    }
  }
//}}}
//{{{
void arrBemf (bool up) {

  if (sixStep.mPrevStep != sixStep.mStep) {
    sixStep.mPrevStep = sixStep.mStep;

    if (sixStep.SPEED_VALIDATED) {
      if (cnt_bemf_event > BEMF_CNT_EVENT_MAX)
        startup_bemf_failure = true;

      if (up && !sixStep.BEMF_OK) {
        n_zcr_startup++;
        cnt_bemf_event = 0;
        }
      else if (!sixStep.BEMF_OK)
        cnt_bemf_event++;

      if ((n_zcr_startup >= NUMBER_ZCR) && !sixStep.BEMF_OK) {
        sixStep.BEMF_OK = true;
        n_zcr_startup = 0;
        }
      }

    if (sixStep.VALIDATION_OK) {
      counter_ARR_Bemf = __HAL_TIM_GetCounter (&hTim6);
      __HAL_TIM_SetAutoreload (&hTim6, counter_ARR_Bemf + ARR_LF / 2);

      }
    }
  }
//}}}

//{{{
void setPiParam (cPiParam* piParam) {

  piParam->Reference = sixStep.CW_CCW ? -target_speed : target_speed;

  piParam->Kp_Gain = sixStep.KP;
  piParam->Ki_Gain = sixStep.KI;

  piParam->Lower_Limit_Output = LOWER_OUT_LIMIT;
  piParam->Upper_Limit_Output = UPPER_OUT_LIMIT;

  piParam->Max_PID_Output = false;
  piParam->Min_PID_Output = false;
  }
//}}}
//{{{
int16_t piController (cPiParam* PI_PARAM, int16_t speed_fdb) {

  int32_t Error = PI_PARAM->Reference - speed_fdb;

  // Proportional term computation
  int32_t wProportional_Term = PI_PARAM->Kp_Gain * Error;

  // Integral term computation
  int32_t wIntegral_Term = 0;
  int32_t wIntegral_sum_temp = 0;
  if (PI_PARAM->Ki_Gain == 0)
    sixStep.Integral_Term_sum = 0;
  else {
    wIntegral_Term = PI_PARAM->Ki_Gain * Error;
    wIntegral_sum_temp = sixStep.Integral_Term_sum + wIntegral_Term;
    sixStep.Integral_Term_sum = wIntegral_sum_temp;
    }

  if (sixStep.Integral_Term_sum > KI_DIV * PI_PARAM->Upper_Limit_Output)
    sixStep.Integral_Term_sum = KI_DIV * PI_PARAM->Upper_Limit_Output;

  if (sixStep.Integral_Term_sum < -KI_DIV * PI_PARAM->Upper_Limit_Output)
    sixStep.Integral_Term_sum = -KI_DIV * PI_PARAM->Upper_Limit_Output;

  // WARNING: the below instruction is not MISRA compliant, user should verify
  //          that Cortex-M3 assembly instruction ASR (arithmetic shift right)
  //          is used by the compiler to perform the shifts (instead of LSR logical shift right)
  int32_t wOutput_32 = (wProportional_Term / KP_DIV) + (sixStep.Integral_Term_sum / KI_DIV);

  if (PI_PARAM->Reference > 0) {
    if (wOutput_32 > PI_PARAM->Upper_Limit_Output)
      wOutput_32 = PI_PARAM->Upper_Limit_Output;
    else if (wOutput_32 < PI_PARAM->Lower_Limit_Output)
      wOutput_32 = PI_PARAM->Lower_Limit_Output;
    }
  else {
    if (wOutput_32 < -PI_PARAM->Upper_Limit_Output)
      wOutput_32 = -PI_PARAM->Upper_Limit_Output;
    else if (wOutput_32 > -PI_PARAM->Lower_Limit_Output)
      wOutput_32 = -PI_PARAM->Lower_Limit_Output;
    }

  return (int16_t)wOutput_32;
  }
//}}}

//{{{
void bemfDelayCalc (uint16_t piReference) {

 if (piReference >= 0) {
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
void taskSpeed() {

  if (!sixStep.VALIDATION_OK &&
      ((sixStep.speed_fdbk_filtered > (target_speed)) || (sixStep.speed_fdbk_filtered < (-target_speed)))) {
    //printf ("taskSpeed SPEED_VALIDATED\n");
    sixStep.STATUS = VALIDATION;
    sixStep.SPEED_VALIDATED = true;
    }

  if (sixStep.SPEED_VALIDATED && sixStep.BEMF_OK && !sixStep.CL_READY)
    sixStep.CL_READY = true;

  if (sixStep.VALIDATION_OK) {
    //printf ("taskSpeed VALIDATION_OK\n");
    sixStep.STATUS = RUN;
    uint16_t ref = (uint16_t)piController (&piParam,(int16_t)sixStep.speed_fdbk_filtered);
    if (piParam.Reference < 0)
      ref = -ref;

    //printf ("ref %d\n", ref);
    sixStep.Current_Reference = ref;
    mcNucleoCurrentRefSetValue (sixStep.Current_Reference);
    }

  bemfDelayCalc (piParam.Reference);
  }
//}}}

// callback interface
//{{{
void mcAdcSample (ADC_HandleTypeDef* hAdc) {

  uint16_t value = HAL_ADC_GetValue (hAdc);

  if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) {
    // up counting started
    if ((sixStep.STATUS != START) && (sixStep.STATUS != ALIGNMENT)) {
      switch (sixStep.mStep) {
        //{{{
        case 1:
          sixStep.mBemfInputBuffer[2] = value;

          if (sixStep.demagn_counter < sixStep.demagn_value)
            sixStep.demagn_counter++;
          else if (piParam.Reference >= 0) {
            if (value < sixStep.ADC_BEMF_threshold_DOWN)
              arrBemf (0);
            }
          else if (value > sixStep.ADC_BEMF_threshold_UP) {
            arrBemf (1);
            sixStep.BEMF_Tdown_count = 0;
            }

          break;
        //}}}
        //{{{
        case 2:
          sixStep.mBemfInputBuffer[1] = value;
          if (sixStep.demagn_counter < sixStep.demagn_value)
            sixStep.demagn_counter++;
          else if (piParam.Reference >= 0) {
            if (value > sixStep.ADC_BEMF_threshold_UP) {
              arrBemf (1);
              sixStep.BEMF_Tdown_count = 0;
              }
            }
          else if (value < sixStep.ADC_BEMF_threshold_DOWN)
            arrBemf (0);

          break;
        //}}}
        //{{{
        case 3:
          sixStep.mBemfInputBuffer[0] = value;
          if (sixStep.demagn_counter < sixStep.demagn_value)
            sixStep.demagn_counter++;
          else if (piParam.Reference >= 0) {
            if (value < sixStep.ADC_BEMF_threshold_DOWN)
              arrBemf (0);
            }
          else if (value > sixStep.ADC_BEMF_threshold_UP) {
            arrBemf (1);
            sixStep.BEMF_Tdown_count = 0;
            }

          break;
        //}}}
        //{{{
        case 4:
          sixStep.mBemfInputBuffer[2] = value;

          if (sixStep.demagn_counter >= sixStep.demagn_value)
            sixStep.demagn_counter++;
          else if (piParam.Reference >= 0) {
            if (value > sixStep.ADC_BEMF_threshold_UP) {
              arrBemf (1);
              sixStep.BEMF_Tdown_count = 0;
              }
            }
           else if (value < sixStep.ADC_BEMF_threshold_DOWN)
             arrBemf (0);

         break;
        //}}}
        //{{{
        case 5:
          sixStep.mBemfInputBuffer[1] = value;
          if (sixStep.demagn_counter < sixStep.demagn_value)
            sixStep.demagn_counter++;
          else if (piParam.Reference >= 0) {
            if (value < sixStep.ADC_BEMF_threshold_DOWN)
              arrBemf (0);
            }
          else if (value > sixStep.ADC_BEMF_threshold_UP) {
            arrBemf (1);
            sixStep.BEMF_Tdown_count = 0;
            }

         break;
        //}}}
        //{{{
        case 6:
          sixStep.mBemfInputBuffer[0] = value;
          if (sixStep.demagn_counter < sixStep.demagn_value)
            sixStep.demagn_counter++;
          else if (piParam.Reference >= 0) {
            if (value > sixStep.ADC_BEMF_threshold_UP) {
              arrBemf (1);
              sixStep.BEMF_Tdown_count = 0;
              }
            }
          else if (value < sixStep.ADC_BEMF_threshold_DOWN)
             arrBemf (0);
          break;
        //}}}
        }
      //printf ("%d adcU %d\n", HAL_GetTick(), sixStep.mStep);
      }

    // set chan for next current/pot/vbus/temp ADC reading
    sixStep.adcChanIndex = (sixStep.adcChanIndex+1) % 4;
    mcNucleoAdcChan (sixStep.adcInputAdc[sixStep.adcChanIndex],
                     sixStep.adcInputChan[sixStep.adcChanIndex]);
    }

  else {
    // down counting started
    sixStep.mAdcBuffer[sixStep.adcChanIndex] = value;
    if (sixStep.adcChanIndex == 1) {
      HFBuffer[HFBufferIndex] = value;
      HFBufferIndex = (HFBufferIndex+1) % HFBUFFERSIZE;
      }

    //printf ("%d i:%d %d\n", HAL_GetTick(), sixStep.adcChanIndex, value);

    // set Chan for next bemf ADC reading
    mcNucleoAdcChan (sixStep.bemfInputAdc[sixStep.mBemfIndex],
                     sixStep.bemfInputChan[sixStep.mBemfIndex]);
    }
  }
//}}}
//{{{
void mcTim6Tick() {

  //printf ("mcTim6Tick %d\n", HAL_GetTick());
  // nextStep
  if (!sixStep.mPmwRunning) {
    mcNucleoStartPwm();
    sixStep.mPmwRunning = true;
    }
  ARR_LF = __HAL_TIM_GetAutoreload (&hTim6);

  if (sixStep.mAligned) {
    sixStep.speed_fdbk = mcGetMechSpeedRPM();
    sixStep.demagn_counter = 1;
    if (sixStep.mPrevStep != sixStep.mStep)
      n_zcr_startup = 0;

    if (piParam.Reference >= 0) {
      sixStep.mStep++;
      if (sixStep.mStep > 6)
        sixStep.mStep = 1;
      if (sixStep.CL_READY)
        sixStep.VALIDATION_OK = true;
      }
    else {
      sixStep.mStep--;
      if (sixStep.mStep < 1)
        sixStep.mStep = 6;
      if (sixStep.CL_READY)
        sixStep.VALIDATION_OK = true;
      }
    }

  if (sixStep.VALIDATION_OK) {
    // motorStall detection and speedFeedback error generation
    sixStep.BEMF_Tdown_count++;
    if (sixStep.BEMF_Tdown_count > BEMF_CONSEC_DOWN_MAX)
      speed_fdbk_error = true;
    else
      __HAL_TIM_SetAutoreload (&hTim6, 0xFFFF);
    }

  sixStepTable (sixStep.mStep);
  if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) {
    // step request during downCount, change adc Chan
    mcNucleoAdcChan (sixStep.bemfInputAdc[sixStep.mBemfIndex],
                     sixStep.bemfInputChan[sixStep.mBemfIndex]);
    //printf ("step request during downCount\n");
    }

  if (!sixStep.ARR_OK) // STEP frequency changing
    arrStep();

  speedFilter();
  }
//}}}
//{{{
void mcSysTick() {

  if (sixStep.mAligning && !sixStep.mAligned) {
    //{{{  align motor
    sixStep.STATUS = ALIGNMENT;
    sixStep.mStep = 6;

    hTim6.Init.Period = sixStep.ARR_value;
    hTim6.Instance->ARR = (uint32_t)hTim6.Init.Period;

    mAlignTicks++;
    if (mAlignTicks >= TIME_FOR_ALIGN + 1) {
      printf ("%d aligned\n", HAL_GetTick());
      sixStep.mAligned = true;
      sixStep.STATUS = STARTUP;
      hTim6.Init.Prescaler = sixStep.prescaler_value;
      hTim6.Instance->PSC = hTim6.Init.Prescaler;
      }

    potSpeedTarget();
    }
    //}}}

  if (sixStep.VALIDATION_OK)
    potSpeed();

  if (sixStep.STATUS != SPEEDFBKERROR)
    taskSpeed();
  if (sixStep.VALIDATION_OK)
    mcSetSpeed();

  if (startup_bemf_failure) {
    sixStep.ACCEL >>= 1;
    if (sixStep.ACCEL < MINIMUM_ACC)
      sixStep.ACCEL = MINIMUM_ACC;
    mcStopMotor();
    sixStep.STATUS = STARTUP_BEMF_FAILURE;
    printf ("STARTUP_BEMF_FAILURE\n");

    cnt_bemf_event = 0;
    }

  if (speed_fdbk_error) {
    mcStopMotor();
    sixStep.STATUS = SPEEDFBKERROR;
    printf ("SPEEDFBKERROR\n");
    }
  }
//}}}

// external interface
//{{{
void mcInit() {

  mcNucleoInit();

  sixStep.HF_TIMx_ARR = hTim1.Instance->ARR;
  sixStep.HF_TIMx_PSC = hTim1.Instance->PSC;
  sixStep.HF_TIMx_CCR = hTim1.Instance->CCR1;

  sixStep.LF_TIMx_ARR = hTim6.Instance->ARR;
  sixStep.LF_TIMx_PSC = hTim6.Instance->PSC;

  sixStep.Ireference = STARTUP_CURRENT_REFERENCE;
  sixStep.NUMPOLESPAIRS = NUM_POLE_PAIRS;

  sixStep.ACCEL = ACC;
  sixStep.KP = KP_GAIN;
  sixStep.KI = KI_GAIN;
  sixStep.CW_CCW = TARGET_SPEED < 0;

  mcReset();
  }
//}}}
//{{{
void mcReset() {

  mLastButtonPress = 0;

  sixStep.mMotorRunning = false;
  sixStep.mPmwRunning = false;
  sixStep.mAligning = false;
  sixStep.mAligned = false;
  sixStep.VALIDATION_OK = false;
  sixStep.ARR_OK = false;
  sixStep.BEMF_OK = false;
  sixStep.CL_READY = false;
  sixStep.SPEED_VALIDATED = false;

  sixStep.numberofitemArr = NUMBER_OF_STEPS;
  sixStep.Ireference = STARTUP_CURRENT_REFERENCE;

  sixStep.pulse_value = sixStep.HF_TIMx_CCR;
  sixStep.Speed_target_ramp = MAX_POT_SPEED;
  sixStep.Speed_Ref_filtered = 0;
  sixStep.demagn_value = INITIAL_DEMAGN_DELAY;

  hTim1.Init.Prescaler = sixStep.HF_TIMx_PSC;
  hTim1.Instance->PSC =  sixStep.HF_TIMx_PSC;
  hTim1.Init.Period =    sixStep.HF_TIMx_ARR;
  hTim1.Instance->ARR =  sixStep.HF_TIMx_ARR;
  hTim1.Instance->CCR1 = sixStep.HF_TIMx_CCR;

  hTim6.Init.Prescaler = sixStep.LF_TIMx_PSC;
  hTim6.Instance->PSC =  sixStep.LF_TIMx_PSC;
  hTim6.Init.Period =    sixStep.LF_TIMx_ARR;
  hTim6.Instance->ARR =  sixStep.LF_TIMx_ARR;

  Rotor_poles_pairs = sixStep.NUMPOLESPAIRS;
  sixStep.SYSCLK_frequency = HAL_RCC_GetSysClockFreq();

  mcNucleoSetChanCCR (0,0,0);

  // PC1 -> ADC12_IN7  curr_fdbk2 - 1shunt
  // PB1 -> ADC3_IN1   pot
  // PA1 -> ADC1_IN2   vbus
  // PC2 -> ADC12_IN8  temp
  sixStep.adcChanIndex = 0;
  sixStep.adcInputAdc[0] = &hAdc1;
  sixStep.adcInputChan[0] = ADC_CHANNEL_7;
  sixStep.adcInputAdc[1] = &hAdc3;
  sixStep.adcInputChan[1] = ADC_CHANNEL_1;
  sixStep.adcInputAdc[2] = &hAdc1;
  sixStep.adcInputChan[2] = ADC_CHANNEL_2;
  sixStep.adcInputAdc[3] = &hAdc1;
  sixStep.adcInputChan[3] = ADC_CHANNEL_8;

  // PC3 -> ADC12_IN9  bemf1
  // PA7 -> ADC2_IN4   bemf3
  // PB0 -> ADC3_IN12  bemf2
  sixStep.mBemfIndex = 0;
  sixStep.bemfInputAdc[0] = &hAdc1;
  sixStep.bemfInputChan[0] = ADC_CHANNEL_9;
  sixStep.bemfInputAdc[1] = &hAdc2;
  sixStep.bemfInputChan[1] = ADC_CHANNEL_4;
  sixStep.bemfInputAdc[2] = &hAdc3;
  sixStep.bemfInputChan[2] = ADC_CHANNEL_12;

  sixStep.ADC_BEMF_threshold_UP = BEMF_THRSLD_UP;
  sixStep.ADC_BEMF_threshold_DOWN = BEMF_THRSLD_DOWN;
  sixStep.demagn_counter = 0;

  sixStep.speed_fdbk_filtered = 0;
  sixStep.Integral_Term_sum = 0;
  sixStep.Current_Reference = 0;
  sixStep.Ramp_Start = 0;
  sixStep.speed_fdbk = 0;

  sixStep.BEMF_Tdown_count = 0;   // Reset of the Counter to detect Stop motor condition when a stall condition occurs

  T_single_step = 0;
  T_single_step_first_value = 0;
  Time_vector_tmp = 0;
  Time_vector_prev_tmp = 0;
  delta = 0;

  Mech_Speed_RPM = 0;
  El_Speed_Hz = 0;
  mech_accel_hz = 0;

  constant_k = 0;
  ARR_LF = 0;
  index_array = 1;

  startup_bemf_failure = false;
  speed_fdbk_error = false;

  index_ARR_step = 1;
  n_zcr_startup = 0;
  cnt_bemf_event = 0;

  mAlignTicks = 1;
  speed_sum_sp_filt = 0;
  speed_sum_pot_filt = 0;
  index_pot_filt = 1;
  potent_filtered = 0;
  counter_ARR_Bemf = 0;
  constant_multiplier_tmp = 0;

  HFBufferIndex =0;
  for (uint16_t i = 0; i < HFBUFFERSIZE;i++)
    HFBuffer[i]=0;

  array_completed = false;
  buffer_completed = false;
  for (uint16_t i = 0; i < FILTER_DEEP;i++) {
    speed_tmp_array[i] = 0;
    speed_tmp_buffer[i]= 0;
    }

  sixStep.mPrevStep = 0;
  if (piParam.Reference < 0)
    sixStep.mStep = 1;
  else
    sixStep.mStep = 0;

  target_speed = TARGET_SPEED;
  setPiParam (&piParam);

  mcNucleoCurrentRefStart();
  mcNucleoCurrentRefSetValue (sixStep.Ireference);

  mMotorStartupCount = 1;
  rampMotor();
  }
//}}}

//{{{
int32_t mcGetElSpeedHz() {

  if (__HAL_TIM_GetAutoreload (&hTim6) != 0xFFFF)
    El_Speed_Hz = (int32_t)((sixStep.SYSCLK_frequency) / hTim6.Instance->PSC) / (__HAL_TIM_GetAutoreload (&hTim6) * 6);
  else
    El_Speed_Hz = 0;

  return piParam.Reference < 0 ? -El_Speed_Hz : El_Speed_Hz;
  }
//}}}
//{{{
int32_t mcGetMechSpeedRPM() {

  Mech_Speed_RPM = (int32_t)(mcGetElSpeedHz() *  60 / Rotor_poles_pairs);
  return (Mech_Speed_RPM);
  }
//}}}

//{{{
void mcStartMotor() {

  sixStep.STATUS = START;
  sixStep.mMotorRunning = true;

  HAL_TIM_Base_Start_IT (&hTim6);
  HAL_ADC_Start_IT (&hAdc1);

  mcNucleoLedOn();
  }
//}}}
//{{{
void mcStopMotor() {

  sixStep.STATUS = STOP;
  sixStep.mMotorRunning = false;

  mcNucleoStopPwm();
  hTim1.Instance->CR1 &= ~(TIM_CR1_CEN);
  hTim1.Instance->CNT = 0;
  mcNucleoDisableChan();

  HAL_TIM_Base_Stop_IT (&hTim6);
  HAL_ADC_Stop_IT (&hAdc1);

  mcNucleoCurrentRefStop();
  mcNucleoLedOff();

  mcReset();
  }
//}}}
//{{{
void mcPanic() {

  mcStopMotor();
  sixStep.STATUS = OVERCURRENT;
  printf ("mcPanic\n");
  }
//}}}

//{{{
void mcSetSpeed() {

  int16_t reference_tmp = 0;

  bool change_target_speed = false;
  if (sixStep.Speed_Ref_filtered > sixStep.Speed_target_ramp) {
    if ((sixStep.Speed_Ref_filtered - sixStep.Speed_target_ramp) > ADC_SPEED_TH)
      change_target_speed = true;
    }
  else if ((sixStep.Speed_target_ramp - sixStep.Speed_Ref_filtered) > ADC_SPEED_TH)
    change_target_speed = true;

  if (change_target_speed) {
    sixStep.Speed_target_ramp = sixStep.Speed_Ref_filtered;

    if (sixStep.CW_CCW) {
      reference_tmp = -(sixStep.Speed_Ref_filtered * MAX_POT_SPEED / 4096);
      piParam.Reference = (reference_tmp >=- MIN_POT_SPEED) ? -MIN_POT_SPEED : reference_tmp;
      }
    else {
      reference_tmp = sixStep.Speed_Ref_filtered * MAX_POT_SPEED / 4096;
      piParam.Reference = (reference_tmp <= MIN_POT_SPEED) ? MIN_POT_SPEED : reference_tmp;
      }
    }
  }
//}}}
//{{{
void mcEXTbutton() {

  if (HAL_GetTick() > mLastButtonPress + 200) {
    printf ("mcEXTbutton toggle %d\n", HAL_GetTick() - mLastButtonPress);
    mLastButtonPress = HAL_GetTick();
    if (sixStep.mMotorRunning) {
      printf ("StopMotor\n");
      mcStopMotor();
      }
    else {
      printf ("StartMotor\n");
      mcStartMotor();
      }
    }
  }
//}}}

// callbacks
//{{{
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc) {
  mcAdcSample (hadc);
  }
//}}}
//{{{
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef* htim) {
  if (htim == &hTim6)
    mcTim6Tick();
  }
//}}}
//{{{
void HAL_SYSTICK_Callback() {
  mcSysTick();
  }
//}}}
//{{{
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {
  mcEXTbutton();
  }
//}}}

//{{{
void SystemClock_Config() {

  // init osc
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig (&RCC_OscInitStruct);

  // init CPU, AHB and APB busses clocks
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_2);

  // init periph clocks
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_ADC12 |
                                       RCC_PERIPHCLK_TIM1 | RCC_PERIPHCLK_TIM16;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  HAL_RCCEx_PeriphCLKConfig (&PeriphClkInit);

  // enable Clock Security System
  HAL_RCC_EnableCSS();

  // config Systick interrupt time
  HAL_SYSTICK_Config (HAL_RCC_GetHCLKFreq() / 1000);

  // config Systick
  HAL_SYSTICK_CLKSourceConfig (SYSTICK_CLKSOURCE_HCLK);

  // SysTick_IRQn interrupt configuration
  HAL_NVIC_SetPriority (SysTick_IRQn, 4, 0);
  }
//}}}
//{{{
int main() {

  HAL_Init();
  SystemClock_Config();

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping (NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority (MemoryManagement_IRQn, 0, 0);
  HAL_NVIC_SetPriority (BusFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority (UsageFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority (SVCall_IRQn, 0, 0);
  HAL_NVIC_SetPriority (DebugMonitor_IRQn, 0, 0);
  HAL_NVIC_SetPriority (PendSV_IRQn, 0, 0);
  HAL_NVIC_SetPriority (SysTick_IRQn, 2, 0);

  mcInit();

  while (1) {
    HAL_Delay (100);
    printf ("%d %d %d i:%d p:%d v:%d t:%d 1:%d 2:%d 3:%d\n",
            ARR_LF, counter_ARR_Bemf, piParam.Reference,
            sixStep.mAdcBuffer[0], sixStep.mAdcBuffer[1] ,sixStep.mAdcBuffer[2] ,sixStep.mAdcBuffer[3],
            sixStep.mBemfInputBuffer[0], sixStep.mBemfInputBuffer[1] ,sixStep.mBemfInputBuffer[2]);
    }
  }
//}}}
