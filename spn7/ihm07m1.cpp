// ihm07m1.cpp
//{{{  ihm07m1 pins
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
//              29 PF0                                    vbus-> PA1 30 ->ADC1_IN2
//              31 PF1                                  DAC_Ch<- PA4 32
//              33 VBAT                                  BEMF2-> PB0 34 ->ADC3_IN12
//  ADC12_IN8<- 35 PC2 <-Temperature             Curr_fdbk_PhB-> PC1 36 ->ADC12_IN7
//   ADC1_IN9<- 37 PC3 <-BEMF1                   Curr_fdbk_PhC-> PC0 38
//
// CN10          1 PC9                                           PC8  2
//               3 PB8                                           PC6  4
//               5 PB9                                           PC5  6
//               7 AVDD                                          U5V  8
//               9 GND                                               10
//              11 PA5 GPIO/DAC/PWM                       CPOUT PA12 12 ->TIM1_ETR
//       TIM1<- 13 PA6 DIAG/ENABLE/BKIN1      DIAG/ENABLE/BKIN2 PA11 14
//   ADC2_IN4<- 15 PA7 <-BEMF3                                  PB12 16 -> SPI CS
//              17 PB6                                          PB11 18 -> SPI VCOM
//              19 PC7                                           GND 20
//              21 PA9 ->VH_PWM                          LedRed<-PB2 22
//              23 PA8 ->UH_PWM                   POTENTIOMETER->PB1 24 ->ADC3_IN1
//       TIM2<- 25 PB10 <-Encoder Z/Hall H3            (BEMF3)->PB15 26 removed ->SPI2 MOSI
//              27 PB4 CurrentRef           (DIAG/ENABLE/BKIN1) PB14 28 removed ->lcd power
//              29 PB5 GPIO/DAC/PWM              (GPIO/DAC/PWM) PB13 30 removed ->SPI2 CLK
//       TIM2<- 31 PB3 <-Encoder B/Hall H2                      AGND 32
//              33 PA10 ->WH_PWM                                 PC4 34
//              35 PA2                                               36
//              37 PA3                                               38
//}}}
//{{{  F303re pins
// u PA0  -> ADC1_IN1   curr_fdbk1/A - 3shunt
//   PA1  -> ADC1_IN2   vbus
//   PA4  -> currentRef DAC
// u PA5  -> gpio/dac/pwm
//   PA6  <- DIAG/ENABLE/BKIN1 TIM1 BRKIN
//   PA7  -> ADC2_IN4   bemf 3/C   temp - PA3 ADC1_IN4
//   PA8  -> TIM1 PWM UH
//   PA9  -> TIM1 PWM VH
//   PA10 -> TIM1 PWM WH
//   PA12 <- CPOUT - TIM1 ETR
// u PA15 -> TIM2_CH1   A/H1
//   PB0  -> ADC3_IN12  bemf 2/B   temp - PA2 ADC1_IN3
//   PB1  -> ADC3_IN1   pot
//   PB2  -> RedLed
// u PB3  -> TIM2_CH2   B/H2
//   PB4  -> currentRef TIM16 CH1 PWM
// u PB5  -> gpio/dac/pwm
// u PB10 -> TIM2_CH4   C/H3
// u PB13 -> SPI2 CLK   removed (gpio/dac/pwm)
// u PB14 <- SPI2 CS    removed (DIAG/ENABLE/BKIN1)
// u PB15 -> SPI2 MOSI  removed (ADC4_IN5  bemf 3/C)
// u PC0  -> ADC12_IN6  curr_fdbk3/C - 3shunt
//   PC1  -> ADC12_IN7  curr_fdbk2/B - 1shunt, 3shunt
//   PC2  -> ADC12_IN8  temp
//   PC3  -> ADC12_IN9  bemf 1/A
//   PC10 -> TIM1 PWM Enable_CH1
//   PC11 -> TIM1 PWM Enable_CH2
//   PC12 -> TIM1 PWM Enable_CH3
//   PC13 <- blueButton
//}}}
//{{{  includes
#include "stm32f3xx_nucleo.h"

#include "ihm07m1.h"
#include "sixStepLib.h"
//}}}

//ADC_HandleTypeDef hAdc1;
ADC_HandleTypeDef hAdc2;
ADC_HandleTypeDef hAdc3;

TIM_HandleTypeDef hTim1;
TIM_HandleTypeDef hTim6;
TIM_HandleTypeDef hTim16;
//{{{
extern "C" {
  void ADC1_2_IRQHandler() { HAL_ADC_IRQHandler (&hAdc2); }
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

//ihm07m1
//{{{
void GPIO_Init() {
// config
// PB2  -> redLed
// PC13 <- blueButton + extInt

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // config gpio pin PB2 - output ihm07m1 redLed
  GPIO_InitTypeDef gpioInit;
  gpioInit.Pin = GPIO_PIN_2;
  gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
  gpioInit.Pull = GPIO_NOPULL;
  gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOB, &gpioInit);
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  // config gpio PC13 input interrupt - nucleoF303re blueButton
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
//   PC3  -> ADC12_IN9  bemf1
//   PB0  -> ADC3_IN12  bemf2
//   PA7  -> ADC2_IN4   bemf3

//   PC1  -> ADC12_IN7  currFdbk2 - 1shunt, 3shunt
//   uPA1  -> ADC1_IN2   vbus
//   PC2  -> ADC12_IN8  temp
//   PB1  -> ADC3_IN1   pot

  //{{{  config clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  __HAL_RCC_ADC12_CLK_ENABLE();
  __HAL_RCC_ADC34_CLK_ENABLE();
  //}}}
  //{{{  config PA7 analog input pin
  GPIO_InitTypeDef gpioInit;
  gpioInit.Pin = GPIO_PIN_7;
  gpioInit.Mode = GPIO_MODE_ANALOG;
  gpioInit.Pull = GPIO_NOPULL;

  HAL_GPIO_Init (GPIOA, &gpioInit);
  //}}}
  //{{{  config PB0 PB1 analog input pin
  gpioInit.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  HAL_GPIO_Init (GPIOB, &gpioInit);
  //}}}
  //{{{  config PC1 PC2 PC3 analog input pin
  gpioInit.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  HAL_GPIO_Init (GPIOC, &gpioInit);
  //}}}

  ADC_ChannelConfTypeDef channelConfig;
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
  //{{{  config PC1 curr  - adc12 chan 7
  channelConfig.Channel = ADC_CHANNEL_7;
  channelConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel (&hAdc2, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");
  //}}}
  //{{{  config PC2 temp  - adc12 chan 8
  channelConfig.Channel = ADC_CHANNEL_8;
  channelConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;

  if (HAL_ADC_ConfigChannel (&hAdc2, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");
  //}}}
  //{{{  config PC3 bemf1 - adc12 chan 9
  channelConfig.Channel = ADC_CHANNEL_9;
  channelConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;

  if (HAL_ADC_ConfigChannel (&hAdc2, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");
  //}}}
  //{{{  config PA7 bemf3 - adc2  chan 4
  channelConfig.Channel = ADC_CHANNEL_4;
  channelConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  channelConfig.Rank = 1;
  channelConfig.SingleDiff = ADC_SINGLE_ENDED;
  channelConfig.OffsetNumber = ADC_OFFSET_NONE;
  channelConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel (&hAdc2, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel2 failed\n");
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
  //{{{  config PB1 pot   - adc3  chan 1
  channelConfig.Rank = 1;
  channelConfig.SingleDiff = ADC_SINGLE_ENDED;
  channelConfig.OffsetNumber = ADC_OFFSET_NONE;
  channelConfig.Channel = ADC_CHANNEL_1;
  channelConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  channelConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel (&hAdc3, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel3 failed\n");
  //}}}
  //{{{  config PB0 bemf2 - adc3  chan 12
  channelConfig.Channel = ADC_CHANNEL_12;
  channelConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  if (HAL_ADC_ConfigChannel (&hAdc3, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel3 failed\n");
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

  hTim1.Instance = TIM1;
  hTim1.Init.Prescaler = 0;
  hTim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  hTim1.Init.Period = 719;
  hTim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  hTim1.Init.RepetitionCounter = 0;
  hTim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init (&hTim1) != HAL_OK)
    printf ("HAL_TIM_Base_Init failed\n");

  // TIM1 config ETR clockSource
  TIM_ClockConfigTypeDef clockSourceConfig;
  clockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource (&hTim1, &clockSourceConfig) != HAL_OK)
    printf ("HAL_TIM_ConfigClockSource failed\n");

  if (HAL_TIM_PWM_Init (&hTim1) != HAL_OK)
    printf ("HAL_TIM_PWM_Init failed\n");

  // TIM1 config ETR clearInput
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

  // TIM1 config master synchronisation
  TIM_MasterConfigTypeDef masterConfig;
  masterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  masterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  masterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization (&hTim1, &masterConfig) != HAL_OK)
    printf ("HAL_TIMEx_MasterConfigSynchronization failed\n");

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
// currentRef PB4 TIM16 CH1 PWM

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

  // config PB4
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
void mcNucleoAdcChan (ADC_HandleTypeDef* adc, uint32_t chan) {

  // stop and wait
  adc->Instance->CR |= ADC_CR_ADSTP;
  while (adc->Instance->CR & ADC_CR_ADSTP);

  // clear old SQx bits, set SQx bits for the selected chan
  adc->Instance->SQR1 &= ~__HAL_ADC_SQR1_RK (ADC_SQR2_SQ5, 1);
  adc->Instance->SQR1 |= __HAL_ADC_SQR1_RK (chan, 1);
  adc->Instance->CR |= ADC_CR_ADSTART;
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
