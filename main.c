// main.c
//{{{  F303RE pins
// CN7
//              1  PC10->Enable_CH1-L6230    Enable_CH2-L6230<-PC11 2
//              3  PC12->Enable_CH3-L6230                      PD2  4
//              5  VDD                                         E5V  6
//              7  BOOT0                                       GND  8
//              9                                                  10
//             11                                            IOREF 12
//             13 PA13                                         RST 14
//             15 PA14                                        +3.3 16
//             17 PA15<-Encoder A/Hall H1         Encoder/Hall<-+5 18
//             19 GND                                          GND 20
//             21 PB7                                          GND 22
//             23 PC13<-BlueButton                             VIN 24
//             25 PC14                                             26
//             27 PC15                          Curr_fdbk_PhA->PA0 28
//             29 PF0                            VBUS_sensing->PA1 30 -> ADC1_IN2
//             31 PF1                                  DAC_Ch->PA4 32
//             33 VBAT                          BEMF2_sensing->PB0 34 -> ADC1_IN11
// ADC1_IN8 <- 35 PC2<-Temperature feedback     Curr_fdbk_PhB->PC1 36 -> ADC1_IN7
// ADC1_IN9 <- 37 PC3<-BEMF1_sensing            Curr_fdbk_PhC->PC0 38
//
// CN10
//               1  PC9                                        PC8  2
//               3  PB8                                        PC6  4
//               5  PB9                                        PC5  6
//               7  AVDD                                       U5V  8
//               9  GND                                            10
//              11  PA5 GPIO/DAC/PWM                    CPOUT PA12 12 -> TIM1
//      TIM1 <- 13  PA6 DIAG/ENABLE/BKIN1   DIAG/ENABLE/BKIN2 PA11 14
// ADC1_IN15 <- 15  PA7<-BEMF3_sensing                        PB12 16
//              17  PB6                                       PB11 18
//              19  PC7                                        GND 20
//              21  PA9->VH_PWM                        LedRed<-PB2 22
//              23  PA8->UH_PWM                 POTENTIOMETER->PB1 24 -> ADC1_IN12
//      TIM2 <- 25 PB10<-Encoder Z/Hall H3     BEMF3_sensing->PB15 26
//              27  PB4 CurrentRef          DIAG/ENABLE/BKIN1 PB14 28
//              29  PB5 GPIO/DAC/PWM             GPIO/DAC/PWM PB13 30
//      TIM2 <- 31  PB3<-Encoder B/Hall H2                    AGND 32
//              33 PA10->WH_PWM                                PC4 34
//              35  PA2                                            36
//              37  PA3                                            38
//}}}
//{{{  includes
#include "stm32f3xx_nucleo.h"
#include "6StepLib.h"
//}}}
//{{{  globals
ADC_HandleTypeDef hadc1;
DAC_HandleTypeDef hdac;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;
UART_HandleTypeDef huart2;

extern SIXSTEP_Base_InitTypeDef SIXSTEP_parameters;
//}}}

// irq handlers
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
void USART2_IRQHandler() {

  HAL_UART_IRQHandler (&huart2);

  #ifdef UART_COMM
    UART_Set_Value();
  #endif
  }
//}}}
//{{{
void TIM1_BRK_TIM15_IRQHandler() {

  printf ("TIM1_BRK_TIM15_IRQHandler\n");

  if (__HAL_TIM_GET_FLAG (&htim1, TIM_FLAG_BREAK) != RESET) {
    MC_StopMotor();
    SIXSTEP_parameters.STATUS = OVERCURRENT;
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

//{{{
void _Error_Handler (char* file, int line) {
  printf ("Error %s %n\n", file, line);
  while(1) {
    }
  }
//}}}

//{{{
void HAL_MspInit() {

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping (NVIC_PRIORITYGROUP_4);

  // MemoryManagement_IRQn interrupt configuration
  HAL_NVIC_SetPriority (MemoryManagement_IRQn, 0, 0);

  // BusFault_IRQn interrupt configuration
  HAL_NVIC_SetPriority (BusFault_IRQn, 0, 0);

  // UsageFault_IRQn interrupt configuration
  HAL_NVIC_SetPriority (UsageFault_IRQn, 0, 0);

  // SVCall_IRQn interrupt configuration
  HAL_NVIC_SetPriority (SVCall_IRQn, 0, 0);

  // DebugMonitor_IRQn interrupt configuration
  HAL_NVIC_SetPriority (DebugMonitor_IRQn, 0, 0);

  // PendSV_IRQn interrupt configuration
  HAL_NVIC_SetPriority (PendSV_IRQn, 0, 0);

  // SysTick_IRQn interrupt configuration
  HAL_NVIC_SetPriority (SysTick_IRQn, 2, 0);
  }
//}}}
//{{{
void HAL_ADC_MspInit (ADC_HandleTypeDef* hadc) {
// ADC1 GPIO Configuration
// PC1 > ADC1_IN7
// PC2 > ADC1_IN8
// PC3 > ADC1_IN9
// PA1 > ADC1_IN2
// PA7 > ADC1_IN15
// PB0 > ADC1_IN11
// PB1 > ADC1_IN12

  if (hadc->Instance == ADC1) {
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
    }
  }
//}}}
//{{{
void HAL_ADC_MspDeInit (ADC_HandleTypeDef* hadc) {

  if (hadc->Instance==ADC1) {
    __HAL_RCC_ADC1_CLK_DISABLE();
    HAL_GPIO_DeInit (GPIOC, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    HAL_GPIO_DeInit (GPIOA, GPIO_PIN_1 | GPIO_PIN_7);
    HAL_GPIO_DeInit (GPIOB, GPIO_PIN_0 | GPIO_PIN_1);
    HAL_NVIC_DisableIRQ (ADC1_IRQn);
    }
  }
//}}}
//{{{
void HAL_TIM_Base_MspInit (TIM_HandleTypeDef* htim_base) {

  if (htim_base->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_ENABLE();

    // config GPIO PA6 TIM1_BKIN timer output
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // config GPIO PA12 TIM1_ETR timer output
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_TIM1;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
    }

  else if (htim_base->Instance == TIM2) {
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
    }

  else if (htim_base->Instance == TIM6) {
    // config TIM6 interrupt
    __HAL_RCC_TIM6_CLK_ENABLE();
    HAL_NVIC_SetPriority (TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ (TIM6_DAC_IRQn);
    }

  else if (htim_base->Instance == TIM16)
    // config tIM16 clock
    __HAL_RCC_TIM16_CLK_ENABLE();
  }
//}}}
//{{{
void HAL_TIM_MspPostInit (TIM_HandleTypeDef* htim) {

  if (htim->Instance == TIM1) {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

  else if (htim->Instance == TIM2) {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

  else if (htim->Instance == TIM16) {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM16;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
  }
//}}}
//{{{
void HAL_TIM_Base_MspDeInit (TIM_HandleTypeDef* htim_base) {

  if (htim_base->Instance==TIM1) {
    __HAL_RCC_TIM1_CLK_DISABLE();
    HAL_GPIO_DeInit (GPIOA, GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_12);
    }

  else if (htim_base->Instance==TIM2) {
    __HAL_RCC_TIM2_CLK_DISABLE();
    HAL_GPIO_DeInit (GPIOA, GPIO_PIN_5);
    HAL_GPIO_DeInit (GPIOB, GPIO_PIN_10|GPIO_PIN_3);
    }

  else if(htim_base->Instance==TIM6) {
    __HAL_RCC_TIM6_CLK_DISABLE();
    /* HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn); */
    }

  else if(htim_base->Instance==TIM16)
    __HAL_RCC_TIM16_CLK_DISABLE();
  }
//}}}
//{{{
void HAL_DAC_MspInit (DAC_HandleTypeDef* hdac) {
// DAC GPIO Configuration
// PA4 > DAC_OUT1

  if (hdac->Instance==DAC) {
    __HAL_RCC_DAC1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // DAC interrupt Init
    HAL_NVIC_SetPriority (TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ (TIM6_DAC_IRQn);
    }
  }
//}}}
//{{{
void HAL_DAC_MspDeInit (DAC_HandleTypeDef* hdac) {

  if (hdac->Instance == DAC) {
    __HAL_RCC_DAC1_CLK_DISABLE();
    HAL_GPIO_DeInit (GPIOA, GPIO_PIN_4);
    /* HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn); */
    }
  }
//}}}
//{{{
void HAL_UART_MspInit (UART_HandleTypeDef* huart) {
// USART2 GPIO Configuration
// PA2 > USART2_TX
// PA3 > USART2_RX

  if (huart->Instance == USART2) {
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

    // USART2 interrupt Init
    HAL_NVIC_SetPriority (USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (USART2_IRQn);
    }
  }
//}}}
//{{{
void HAL_UART_MspDeInit (UART_HandleTypeDef* huart) {

  if (huart->Instance == USART2) {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit (GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
    HAL_NVIC_DisableIRQ (USART2_IRQn);
    }
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
  if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);

  // init CPU, AHB and APB busses clocks
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);

  // init periph clocks
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_ADC12 |
                                       RCC_PERIPHCLK_TIM1 | RCC_PERIPHCLK_TIM16;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInit) != HAL_OK)
    _Error_Handler (__FILE__, __LINE__);

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
static void MX_GPIO_Init() {

  // GPIO Ports Clock Enable
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure GPIO pin Output Level
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_4 | GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);

  // Configure GPIO pin Output Level
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  // Configure GPIO PC13 input interrupt - blueButton
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);

  // Configure GPIO PC4 PC7 PC10 PC11 PC12 output
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);

  // Configure GPIO pin PB2 - output ihm07m1 red led
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
static void MX_ADC1_Init() {

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
static void MX_TIM1_Init() {

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

  HAL_TIM_MspPostInit (&htim1);
  }
//}}}
//{{{
static void MX_TIM2_Init() {

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

  HAL_TIM_MspPostInit (&htim2);
  }
//}}}
//{{{
static void MX_TIM6_Init() {

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
static void MX_TIM16_Init() {

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

  HAL_TIM_MspPostInit(&htim16);
  }
//}}}
//{{{
static void MX_DAC_Init() {

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
static void MX_USART2_UART_Init() {

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_ODD;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init (&huart2) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);
  }
//}}}

//{{{
int main() {

  HAL_Init();
  SystemClock_Config();

  BSP_LED_Init (LED2);
  BSP_LED_On (LED2);
  BSP_LED_Off (LED2);
  BSP_LED_On (LED2);

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_DAC_Init();
  MX_USART2_UART_Init();

  MC_SixStep_INIT();

  int loop = 0;
  while (1) {
    HAL_Delay (1000);
    BSP_LED_Off (LED2);
    HAL_Delay (1000);
    BSP_LED_On (LED2);
    printf ("loop %d\n", loop++);
    }
  }
//}}}
