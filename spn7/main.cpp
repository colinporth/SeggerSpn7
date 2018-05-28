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
//              31 PF1                                  DAC_Ch-> PA4 32
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

ADC_HandleTypeDef hAdc1;
TIM_HandleTypeDef hTim1;
TIM_HandleTypeDef hTim2;
TIM_HandleTypeDef hTim6;
TIM_HandleTypeDef hTim16;
DAC_HandleTypeDef hDac;

cSixStep sixStep;
cPiParam piParam;
//{{{  vars
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
uint16_t HFBuffer[HFBUFFERSIZE];          //  Buffer for Potentiometer Value Filtering at the High-Frequency ADC conversion
uint16_t HFBufferIndex = 0;               //  High-Frequency Buffer Index
bool array_completed = false;             //  Speed filter variable
bool buffer_completed = false;            //  Potentiometer filter variable

uint32_t ARR_LF = 0;                      //  Autoreload LF TIM variable
int32_t Mech_Speed_RPM = 0;               //  Mechanical motor speed
int32_t El_Speed_Hz = 0;                  //  Electrical motor speed

uint16_t index_adc_chn = 0;               //  Index of ADC channel selector for measuring
bool Enable_start_button = true;          //  Start/stop button filter to avoid double command

uint16_t index_ARR_step = 1;
uint32_t n_zcr_startup = 0;
uint16_t index_startup_motor = 1;
uint16_t target_speed = TARGET_SPEED;     //  Target speed for closed loop control
uint16_t shift_n_sqrt = 14;

uint16_t cnt_bemf_event = 0;
bool startup_bemf_failure = 0;

uint8_t speed_fdbk_error = 0;

uint16_t index_align = 1;
int32_t speed_sum_sp_filt = 0;
int32_t speed_sum_pot_filt = 0;
uint16_t index_pot_filt = 1;
int16_t potent_filtered = 0;

uint32_t Tick_cnt = 0;
uint32_t counter_ARR_Bemf = 0;
uint64_t constant_multiplier_tmp = 0;
//}}}

extern "C" {
  //{{{
  void ADC1_IRQHandler() {

    //printf ("ADC1_IRQHandler\n");
    HAL_ADC_IRQHandler (&hAdc1);
    }
  //}}}
  //{{{
  void TIM1_BRK_TIM15_IRQHandler() {

    //printf ("TIM1_BRK_TIM15_IRQHandler\n");
    if (__HAL_TIM_GET_FLAG (&hTim1, TIM_FLAG_BREAK) != RESET)
      MC_Panic();

    HAL_TIM_IRQHandler (&hTim1);
    }
  //}}}
  //{{{
  void TIM6_DAC_IRQHandler() {

    //printf ("TIM6_DAC_IRQHandler\n");
    HAL_TIM_IRQHandler (&hTim6);
    HAL_DAC_IRQHandler (&hDac);
    }
  //}}}
  //{{{
  void EXTI15_10_IRQHandler() {

    printf ("EXTI15_10_IRQHandler\n");
    HAL_GPIO_EXTI_IRQHandler (GPIO_PIN_13);
    }
  //}}}
  //{{{
  void SysTick_Handler() {

    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
    }
  //}}}
  }

//{{{  ihm07m1
//{{{
void MC_ADC_Channel (uint32_t adc_ch) {

  hAdc1.Instance->CR |= ADC_CR_ADSTP;
  while (hAdc1.Instance->CR & ADC_CR_ADSTP);

  // Clear the old SQx bits for the selected rank
  hAdc1.Instance->SQR1 &= ~__HAL_ADC_SQR1_RK (ADC_SQR2_SQ5, 1);

  // Set the SQx bits for the selected rank
  hAdc1.Instance->SQR1 |= __HAL_ADC_SQR1_RK (adc_ch, 1);
  hAdc1.Instance->CR |= ADC_CR_ADSTART;
  }
//}}}

//{{{
void MC_DisableInput_CH1_D_CH2_D_CH3_D() {

  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);  // EN1 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);  // EN2 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);  // EN3 DISABLE
  }
//}}}
//{{{
void MC_EnableInput_CH1_E_CH2_E_CH3_D() {

  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_10, GPIO_PIN_SET);    // EN1 ENABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_11, GPIO_PIN_SET);    // EN2 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);  // EN3 ENABLE
  }
//}}}
//{{{
void MC_EnableInput_CH1_E_CH2_D_CH3_E() {

  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_10, GPIO_PIN_SET);   // EN1 ENABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // EN2 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_12, GPIO_PIN_SET);   // EN3 ENABLE
  }
//}}}
//{{{
void MC_EnableInput_CH1_D_CH2_E_CH3_E() {

  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // EN1 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_11, GPIO_PIN_SET);   // EN2 ENABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_12, GPIO_PIN_SET);   // EN3 ENABLE
  }
//}}}
//{{{
void MC_TIM1_CH1_SetCCR (uint16_t value) {
  hTim1.Instance->CCR1 = value;
  }
//}}}
//{{{
void MC_TIM1_CH2_SetCCR (uint16_t value) {
  hTim1.Instance->CCR2 = value;
  }
//}}}
//{{{
void MC_TIM1_CH3_SetCCR (uint16_t value) {
  hTim1.Instance->CCR3 = value;
  }
//}}}
//{{{
void MC_Start_PWM() {
  HAL_TIM_PWM_Start (&hTim1, TIM_CHANNEL_1); // TIM1_CH1 ENABLE
  HAL_TIM_PWM_Start (&hTim1, TIM_CHANNEL_2); // TIM1_CH2 ENABLE
  HAL_TIM_PWM_Start (&hTim1, TIM_CHANNEL_3); // TIM1_CH3 ENABLE
  }
//}}}
//{{{
void MC_Stop_PWM() {
  HAL_TIM_PWM_Stop (&hTim1, TIM_CHANNEL_1); // TIM1_CH1 DISABLE
  HAL_TIM_PWM_Stop (&hTim1, TIM_CHANNEL_2); // TIM1_CH2 DISABLE
  HAL_TIM_PWM_Stop (&hTim1, TIM_CHANNEL_3); // TIM1_CH3 DISABLE
  }
//}}}

//{{{
void MC_CurrentRefStart() {

  hTim16.Instance->CCR1 = 0;
  HAL_TIM_PWM_Start (&hTim16, TIM_CHANNEL_1);
  }
//}}}
//{{{
void MC_CurrentRefStop() {

  hTim16.Instance->CCR1 = 0;
  HAL_TIM_PWM_Stop (&hTim16, TIM_CHANNEL_1);
  }
//}}}
//{{{
void MC_CurrentRefSetValue (uint16_t value) {
  printf ("MC_Current_Reference_Setvalue %d\n", value);
  hTim16.Instance->CCR1 = (uint32_t)(value * hTim16.Instance->ARR) / 4096;
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
//}}}
//{{{  ihm07m1 inits
//{{{
void GPIO_Init() {
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
void ADC1_Init() {
// PC1 -> ADC1_IN7   current
// PC2 -> ADC1_IN8   temp
// PC3 -> ADC1_IN9   bemf1
// PA1 -> ADC1_IN2   vbus
// PA7 -> ADC1_IN15  bemf3
// PB0 -> ADC1_IN11  bemf2
// PB1 -> ADC1_IN12  speed

  __HAL_RCC_ADC1_CLK_ENABLE();

  // config GPIO C adc inputs
  GPIO_InitTypeDef gpioInit;
  gpioInit.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  gpioInit.Mode = GPIO_MODE_ANALOG;
  gpioInit.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOC, &gpioInit);

  // config GPIO A adc inputs
  gpioInit.Pin = GPIO_PIN_1 | GPIO_PIN_7;
  gpioInit.Mode = GPIO_MODE_ANALOG;
  gpioInit.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOA, &gpioInit);

  // config GPIO B adc inputs
  gpioInit.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpioInit.Mode = GPIO_MODE_ANALOG;
  gpioInit.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOB, &gpioInit);

  // ADC1 interrupt Init
  HAL_NVIC_SetPriority (ADC1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (ADC1_IRQn);

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

  // Current feedback
  ADC_ChannelConfTypeDef channelConfig;
  channelConfig.Channel = ADC_Current;
  channelConfig.Rank = 1;
  channelConfig.SingleDiff = ADC_SINGLE_ENDED;
  channelConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  channelConfig.OffsetNumber = ADC_OFFSET_NONE;
  channelConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel (&hAdc1, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");

  // Potentiometer
  channelConfig.Channel = ADC_Pot;
  channelConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  if (HAL_ADC_ConfigChannel (&hAdc1, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");

  // Bus voltage
  channelConfig.Channel = ADC_Vbus;
  channelConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  if (HAL_ADC_ConfigChannel (&hAdc1, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");

  // Temperature feedback
  channelConfig.Channel = ADC_Temp;
  channelConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  if (HAL_ADC_ConfigChannel (&hAdc1, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");

  // BEMF feedback phase A
  channelConfig.Channel = ADC_Bemf_CH1;
  channelConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  if (HAL_ADC_ConfigChannel (&hAdc1, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");

  // BEMF feedback phase B
  channelConfig.Channel = ADC_Bemf_CH2;
  channelConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  if (HAL_ADC_ConfigChannel (&hAdc1, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");

  // BEMF feedback phase C
  channelConfig.Channel = ADC_Bemf_CH3;
  channelConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  if (HAL_ADC_ConfigChannel (&hAdc1, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel failed\n");
  }
//}}}
//{{{
void TIM1_Init() {
// 3 phase PWM

  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  //{{{  config PA6 TIM1_BKIN timer output
  GPIO_InitTypeDef GPIO_InitStruct;
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
  //{{{  config PC10 PC11 PC12 output
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);
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
  TIM_ClockConfigTypeDef sClockSourceConfig;
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource (&hTim1, &sClockSourceConfig) != HAL_OK)
    printf ("HAL_TIM_ConfigClockSource failed\n");
  if (HAL_TIM_PWM_Init (&hTim1) != HAL_OK)
    printf ("HAL_TIM_PWM_Init failed\n");
  //}}}
  //{{{  TIM1 clearInput ETR
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
  //{{{  TIM1 masterConfig
  TIM_MasterConfigTypeDef masterConfig;
  masterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  masterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  masterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization (&hTim1, &masterConfig) != HAL_OK)
    printf ("HAL_TIMEx_MasterConfigSynchronization failed\n");
  //}}}
  //{{{  TIM1 configOC
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
  //{{{  TIM1 deadTime
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
  //{{{  config PA8 PA9 PA10 PWM YUW out, why late?
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
  //}}}

  // stop TIM during Breakpoint
  __HAL_FREEZE_TIM1_DBGMCU();
  __HAL_TIM_ENABLE_IT (&hTim1, TIM_IT_BREAK);
  }
//}}}
//{{{
void TIM2_Init() {
// Hall effect ?
// PB3  -> TIM2_CH2
// PB10 -> TIM2_CH3
// PA5  -> ?

  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // config PB3 PB10 timer outputs
  GPIO_InitTypeDef gpioInit;
  gpioInit.Pin = GPIO_PIN_3 | GPIO_PIN_10;
  gpioInit.Mode = GPIO_MODE_AF_PP;
  gpioInit.Pull = GPIO_NOPULL;
  gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
  gpioInit.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init (GPIOB, &gpioInit);

  hTim2.Instance = TIM2;
  hTim2.Init.Prescaler = 719;
  hTim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  hTim2.Init.Period = 50000;
  hTim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  hTim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init (&hTim2) != HAL_OK)
    printf ("HAL_TIM_Base_Init failed\n");

  TIM_ClockConfigTypeDef clockSourceConfig;
  clockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource (&hTim2, &clockSourceConfig) != HAL_OK)
    printf ("HAL_TIM_ConfigClockSource failed\n");
  if (HAL_TIM_PWM_Init (&hTim2) != HAL_OK)
    printf ("HAL_TIM_PWM_Init failed\n");

  TIM_MasterConfigTypeDef masterConfig;
  masterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  masterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization (&hTim2, &masterConfig) != HAL_OK)
    printf ("HAL_TIMEx_MasterConfigSynchronization failed\n");

  TIM_OC_InitTypeDef configOC;
  configOC.OCMode = TIM_OCMODE_PWM1;
  configOC.Pulse = 0;
  configOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  configOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel (&hTim2, &configOC, TIM_CHANNEL_1) != HAL_OK)
    printf ("HAL_TIM_PWM_ConfigChannel failed\n");

  // config PA5
  gpioInit.Pin = GPIO_PIN_5;
  gpioInit.Mode = GPIO_MODE_AF_PP;
  gpioInit.Pull = GPIO_NOPULL;
  gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
  gpioInit.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init (GPIOA, &gpioInit);
  }
//}}}
//{{{
void TIM6_Init() {

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
void DAC_Init() {
// PA4 -> DAC_OUT1

  __HAL_RCC_DAC1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef gpioInit;
  gpioInit.Pin = GPIO_PIN_4;
  gpioInit.Mode = GPIO_MODE_ANALOG;
  gpioInit.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOA, &gpioInit);

  hDac.Instance = DAC;
  if (HAL_DAC_Init (&hDac) != HAL_OK)
    printf ("HAL_DAC_Init failed\n");

  DAC_ChannelConfTypeDef config;
  config.DAC_Trigger = DAC_TRIGGER_NONE;
  config.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel (&hDac, &config, DAC_CHANNEL_1) != HAL_OK)
    printf ("HAL_DAC_ConfigChannel failed\n");

  // interrupt Init
  HAL_NVIC_SetPriority (TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ (TIM6_DAC_IRQn);
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

  target_speed = sixStep.ADC_Regular_Buffer[1] * MAX_POT_SPEED / 4096;

  if (target_speed < MIN_POT_SPEED)
    target_speed = MIN_POT_SPEED;

  if (target_speed > (MAX_POT_SPEED / VAL_POT_SPEED_DIV))
    target_speed = (MAX_POT_SPEED/VAL_POT_SPEED_DIV);

  printf ("potSpeedTarget %d\n", target_speed);
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

  printf ("potSpeed %d\n", potent_filtered);
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
      MC_TIM1_CH1_SetCCR (sixStep.pulse_value);
      MC_TIM1_CH2_SetCCR (0);
      MC_TIM1_CH3_SetCCR (0);
      MC_EnableInput_CH1_E_CH2_E_CH3_D();
      sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[3];
      break;

    case 2:
      MC_TIM1_CH1_SetCCR (sixStep.pulse_value);
      MC_TIM1_CH2_SetCCR (0);
      MC_TIM1_CH3_SetCCR (0);
      MC_EnableInput_CH1_E_CH2_D_CH3_E();
      sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[2];
      break;

    case 3:
      MC_TIM1_CH2_SetCCR (sixStep.pulse_value);
      MC_TIM1_CH1_SetCCR (0);
      MC_TIM1_CH3_SetCCR (0);
      MC_EnableInput_CH1_D_CH2_E_CH3_E();
      sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[1];
      break;

    case 4:
      MC_TIM1_CH2_SetCCR (sixStep.pulse_value);
      MC_TIM1_CH1_SetCCR (0);
      MC_TIM1_CH3_SetCCR (0);
      MC_EnableInput_CH1_E_CH2_E_CH3_D();
      sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[3];
     break;

    case 5:
      MC_TIM1_CH3_SetCCR (sixStep.pulse_value);
      MC_TIM1_CH1_SetCCR (0);
      MC_TIM1_CH2_SetCCR (0);
      MC_EnableInput_CH1_E_CH2_D_CH3_E();
      sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[2];
      break;

    case 6:
      MC_TIM1_CH3_SetCCR (sixStep.pulse_value);
      MC_TIM1_CH2_SetCCR (0);
      MC_TIM1_CH1_SetCCR (0);
      MC_EnableInput_CH1_D_CH2_E_CH3_E();
      sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[1];
      break;
     }
  }
//}}}
//{{{
void rampMotorCalc() {

  uint32_t constant_multiplier = 100;
  uint32_t constant_multiplier_2 = 4000000000;

  if (index_startup_motor) {
    mech_accel_hz = sixStep.ACCEL * Rotor_poles_pairs / 60;
    constant_multiplier_tmp = (uint64_t)constant_multiplier * (uint64_t)constant_multiplier_2;
    constant_k = constant_multiplier_tmp / (3 * mech_accel_hz);
    MC_CurrentRefSetValue (sixStep.Ireference);
    Time_vector_prev_tmp = 0;
    }

  if (index_startup_motor < NUMBER_OF_STEPS) {
    Time_vector_tmp = ((uint64_t)1000 * (uint64_t)1000 * (uint64_t) fastSqrt(((uint64_t)index_startup_motor * (uint64_t)constant_k)))/632455;
    delta = Time_vector_tmp - Time_vector_prev_tmp;
    if (index_startup_motor) {
      T_single_step_first_value = (2 * 3141) * delta / 1000;
      sixStep.ARR_value = (uint32_t)(65535);
      }
    else {
      T_single_step = (2 * 3141)* delta / 1000;
      sixStep.ARR_value = (uint32_t)(65535 * T_single_step) / T_single_step_first_value;
      }
    }
  else
    index_startup_motor = 1;

  if (index_startup_motor)
    sixStep.prescaler_value = (((sixStep.SYSCLK_frequency / 1000000) * T_single_step_first_value)/65535) - 1;

  if ((sixStep.STATUS != ALIGNMENT) && (sixStep.STATUS != START))
    index_startup_motor++;
  else
    Time_vector_tmp = 0;

  Time_vector_prev_tmp = Time_vector_tmp;
  }
//}}}

//{{{
void nextStep() {

  if (sixStep.CMD) {
    sixStep.CMD = false;
    MC_Start_PWM();
    }
  ARR_LF = __HAL_TIM_GetAutoreload (&hTim6);

  if (sixStep.ALIGN_OK) {
    sixStep.speed_fdbk = MC_GetMechSpeedRPM();
    sixStep.demagn_counter = 1;
    if (sixStep.prev_step_position != sixStep.step_position)
      n_zcr_startup = 0;

    if (piParam.Reference >= 0) {
      sixStep.step_position++;
      if (sixStep.step_position > 6)
        sixStep.step_position = 1;
      if (sixStep.CL_READY)
        sixStep.VALIDATION_OK = true;
      }
    else {
      sixStep.step_position--;
      if (sixStep.step_position < 1)
        sixStep.step_position = 6;
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
  sixStepTable (sixStep.step_position);

  // It controls if the changing step request appears during DOWNcounting
  // in this case it changes the ADC channel UP-COUNTING direction started DIR = 0
  if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) {
    switch (sixStep.step_position) {
      case 3:
      case 6: sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[1]; break;
      case 2:
      case 5: sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[2]; break;
      case 1:
      case 4: sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[3]; break;
      }

    MC_ADC_Channel (sixStep.CurrentRegular_BEMF_ch);
    }
  }
//}}}
//{{{
void arrStep() {

  if (!sixStep.ALIGNMENT)
    sixStep.ALIGNMENT = true;

  if (sixStep.ALIGN_OK) {
    if (piParam.Reference >= 0) {
      if (!sixStep.VALIDATION_OK) {
        sixStep.STATUS = STARTUP;
        rampMotorCalc();
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
          MC_StopMotor();
          sixStep.STATUS = STARTUP_FAILURE;
          }
        }
      else {
        sixStep.ARR_OK = true;
        index_startup_motor = 1;
        index_ARR_step = 1;
        }
      }
    else {
      if (!sixStep.VALIDATION_OK) {
        sixStep.STATUS = STARTUP;
        rampMotorCalc();
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
          MC_StopMotor();
          sixStep.STATUS = STARTUP_FAILURE;
          }
        }
      else {
        sixStep.ARR_OK = true;
        index_startup_motor = 1;
        index_ARR_step = 1;
        }
      }
    }
  }
//}}}
//{{{
void arrBemf (bool up_bemf) {

  if (sixStep.prev_step_position != sixStep.step_position) {
    if (sixStep.SPEED_VALIDATED) {
       if (cnt_bemf_event > BEMF_CNT_EVENT_MAX)
        startup_bemf_failure = true;

      if (up_bemf && !sixStep.BEMF_OK) {
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

    sixStep.prev_step_position = sixStep.step_position;

    if (sixStep.VALIDATION_OK) {
      counter_ARR_Bemf = __HAL_TIM_GetCounter (&hTim6);
      __HAL_TIM_SetAutoreload (&hTim6, counter_ARR_Bemf + ARR_LF / 2);
      }
    }
  }
//}}}

//{{{
void setPiParam (cPiParam* PI_PARAM) {

  PI_PARAM->Reference = sixStep.CW_CCW ? -target_speed : target_speed;

  PI_PARAM->Kp_Gain = sixStep.KP;
  PI_PARAM->Ki_Gain = sixStep.KI;

  PI_PARAM->Lower_Limit_Output = LOWER_OUT_LIMIT;
  PI_PARAM->Upper_Limit_Output = UPPER_OUT_LIMIT;

  PI_PARAM->Max_PID_Output = false;
  PI_PARAM->Min_PID_Output = false;
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
      (sixStep.speed_fdbk_filtered > (target_speed) || sixStep.speed_fdbk_filtered < (-target_speed))) {
    printf ("taskSpeed SPEED_VALIDATED\n");
    sixStep.STATUS = VALIDATION;
    sixStep.SPEED_VALIDATED = true;
    }

  if (sixStep.SPEED_VALIDATED && sixStep.BEMF_OK && !sixStep.CL_READY)
    sixStep.CL_READY = true;

  if (sixStep.VALIDATION_OK) {
    printf ("taskSpeed VALIDATION_OK\n");
    sixStep.STATUS = RUN;
    uint16_t ref = (uint16_t)piController (&piParam,(int16_t)sixStep.speed_fdbk_filtered);
    if (piParam.Reference < 0)
      ref = -ref;
    printf ("ref %d\n", ref);
    sixStep.Current_Reference = ref;
    MC_CurrentRefSetValue (sixStep.Current_Reference);
    }

  bemfDelayCalc (piParam.Reference);
  }
//}}}

// callback interface
//{{{
void MC_ADC() {

  if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) {
    // UP-counting direction started, GET the ADC value (PHASE CURRENT)
    if ((sixStep.STATUS != START) && (sixStep.STATUS != ALIGNMENT)) {
      switch (sixStep.step_position) {
        //{{{
        case 1:
          if (sixStep.demagn_counter >= sixStep.demagn_value) {
            sixStep.ADC_BUFFER[3] = HAL_ADC_GetValue (&hAdc1);
            if (piParam.Reference >= 0) {
              if (sixStep.ADC_BUFFER[3] < sixStep.ADC_BEMF_threshold_DOWN)
                arrBemf (0);
              }
            else if (sixStep.ADC_BUFFER[3] > sixStep.ADC_BEMF_threshold_UP) {
              arrBemf (1);
              sixStep.BEMF_Tdown_count = 0;
              }
            }
          else
            sixStep.demagn_counter++;

          break;
        //}}}
        //{{{
        case 2:
          if (sixStep.demagn_counter >= sixStep.demagn_value) {
            sixStep.ADC_BUFFER[2] = HAL_ADC_GetValue (&hAdc1);
            if (piParam.Reference >= 0) {
              if (sixStep.ADC_BUFFER[2] > sixStep.ADC_BEMF_threshold_UP) {
                arrBemf (1);
                sixStep.BEMF_Tdown_count = 0;
                }
              }
            else if (sixStep.ADC_BUFFER[2] < sixStep.ADC_BEMF_threshold_DOWN)
              arrBemf (0);
            }
          else
           sixStep.demagn_counter++;

          break;
        //}}}
        //{{{
        case 3:
          if (sixStep.demagn_counter >= sixStep.demagn_value) {
            sixStep.ADC_BUFFER[1] = HAL_ADC_GetValue (&hAdc1);
            if (piParam.Reference >= 0) {
              if (sixStep.ADC_BUFFER[1] < sixStep.ADC_BEMF_threshold_DOWN)
                arrBemf (0);
              }
            else if (sixStep.ADC_BUFFER[1] > sixStep.ADC_BEMF_threshold_UP) {
              arrBemf (1);
              sixStep.BEMF_Tdown_count = 0;
              }
            }
          else
            sixStep.demagn_counter++;

          break;
        //}}}
        //{{{
        case 4:
         if (sixStep.demagn_counter >= sixStep.demagn_value) {
           sixStep.ADC_BUFFER[3] = HAL_ADC_GetValue (&hAdc1);
           if (piParam.Reference >= 0) {
             if (sixStep.ADC_BUFFER[3] > sixStep.ADC_BEMF_threshold_UP) {
               arrBemf (1);
               sixStep.BEMF_Tdown_count = 0;
               }
             }
           else if (sixStep.ADC_BUFFER[3] < sixStep.ADC_BEMF_threshold_DOWN)
             arrBemf (0);
           }
         else
           sixStep.demagn_counter++;

         break;
        //}}}
        //{{{
        case 5:
          if (sixStep.demagn_counter >= sixStep.demagn_value) {
           sixStep.ADC_BUFFER[2] = HAL_ADC_GetValue (&hAdc1);
           if (piParam.Reference >= 0) {
             if (sixStep.ADC_BUFFER[2] < sixStep.ADC_BEMF_threshold_DOWN)
               arrBemf (0);
             }
           else if (sixStep.ADC_BUFFER[2] > sixStep.ADC_BEMF_threshold_UP) {
             arrBemf (1);
             sixStep.BEMF_Tdown_count = 0;
             }
           }
         else
           sixStep.demagn_counter++;

         break;
        //}}}
        //{{{
        case 6:
          if (sixStep.demagn_counter >= sixStep.demagn_value) {
            sixStep.ADC_BUFFER[1] = HAL_ADC_GetValue (&hAdc1);
            if (piParam.Reference>=0) {
             if (sixStep.ADC_BUFFER[1] > sixStep.ADC_BEMF_threshold_UP) {
               arrBemf (1);
               sixStep.BEMF_Tdown_count = 0;
               }
             }
           else if (sixStep.ADC_BUFFER[1] < sixStep.ADC_BEMF_threshold_DOWN)
             arrBemf (0);
           }
         else
           sixStep.demagn_counter++;
          break;
        //}}}
        }
      //printf ("%d adcU %d\n", HAL_GetTick(), sixStep.step_position);
      }

    // SET ADC CHANNEL FOR SPEED/CURRENT/VBUS
    // Set the channel for next ADC Regular reading
    MC_ADC_Channel (sixStep.ADC_SEQ_CHANNEL[index_adc_chn]);
    }

  else {
    // Down-counting direction started, Set the channel for next ADC Regular reading
    sixStep.ADC_Regular_Buffer[index_adc_chn] = HAL_ADC_GetValue (&hAdc1);
    if (index_adc_chn == 1) {
      HFBuffer[HFBufferIndex++] = HAL_ADC_GetValue (&hAdc1);
      if (HFBufferIndex >= HFBUFFERSIZE)
        HFBufferIndex = 0;
      }

    //printf ("%d adcD %d\n", HAL_GetTick(), index_adc_chn);

    index_adc_chn++;
    if (index_adc_chn > 3)
      index_adc_chn = 0;

    MC_ADC_Channel (sixStep.CurrentRegular_BEMF_ch);
    }
  }
//}}}
//{{{
void MC_Timebase() {

  nextStep();

  // BASE TIMER - ARR modification for STEP frequency changing
  if (!sixStep.ARR_OK)
    arrStep();

  speedFilter();
  }
//}}}
//{{{
void MC_SysTick() {

  if (sixStep.ALIGNMENT && !sixStep.ALIGN_OK) {
    //{{{  align motor
    sixStep.step_position = 6;
    hTim6.Init.Period = sixStep.ARR_value;
    hTim6.Instance->ARR = (uint32_t)hTim6.Init.Period;
    sixStep.STATUS = ALIGNMENT;

    potSpeedTarget();

    index_align++;
    if (index_align >= TIME_FOR_ALIGN + 1) {
      printf ("%d aligned\n", HAL_GetTick());
      sixStep.ALIGN_OK = true;
      sixStep.STATUS = STARTUP;
      index_startup_motor = 1;
      rampMotorCalc();
      hTim6.Init.Prescaler = sixStep.prescaler_value;
      hTim6.Instance->PSC = hTim6.Init.Prescaler;
      index_align = 0;
      }
    }
    //}}}

  if (sixStep.VALIDATION_OK)
    potSpeed();

  // Push button delay time to avoid double command
  if (HAL_GetTick() == BUTTON_DELAY && !Enable_start_button)
    Enable_start_button = true;

  // sixStep.Speed_Loop_Time x 1msec
  if (Tick_cnt >= sixStep.Speed_Loop_Time) {
    if (sixStep.STATUS != SPEEDFBKERROR)
      taskSpeed();
    if (sixStep.VALIDATION_OK)
      MC_SetSpeed();
    Tick_cnt = 0;
    }
  else
    Tick_cnt++;

  if (startup_bemf_failure) {
    sixStep.ACCEL >>= 1;
    if (sixStep.ACCEL < MINIMUM_ACC)
      sixStep.ACCEL = MINIMUM_ACC;
    MC_StopMotor();
    cnt_bemf_event = 0;
    sixStep.STATUS = STARTUP_BEMF_FAILURE;
    }

  if (speed_fdbk_error) {
    MC_StopMotor();
    sixStep.STATUS = SPEEDFBKERROR;
    }
  }
//}}}

// external interface
//{{{
void MC_Init() {

  MC_Nucleo_Init();

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
  sixStep.CW_CCW = DIRECTION;
  sixStep.Button_ready = true;

  MC_Reset();
  }
//}}}
//{{{
void MC_Reset() {

  sixStep.CMD = true;
  sixStep.numberofitemArr = NUMBER_OF_STEPS;
  sixStep.ADC_BEMF_threshold_UP = BEMF_THRSLD_UP;
  sixStep.ADC_BEMF_threshold_DOWN = BEMF_THRSLD_DOWN;
  sixStep.Ireference = STARTUP_CURRENT_REFERENCE;
  sixStep.Speed_Loop_Time = SPEED_LOOP_TIME;
  sixStep.pulse_value = sixStep.HF_TIMx_CCR;
  sixStep.Speed_target_ramp = MAX_POT_SPEED;
  sixStep.ALIGNMENT = false;
  sixStep.Speed_Ref_filtered = 0;
  sixStep.demagn_value = INITIAL_DEMAGN_DELAY;

  sixStep.CurrentRegular_BEMF_ch = 0;
  sixStep.prev_step_position = 0;
  sixStep.step_position = 0;

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

  MC_TIM1_CH1_SetCCR (0);
  MC_TIM1_CH2_SetCCR(0);
  MC_TIM1_CH3_SetCCR(0);

  sixStep.Regular_channel[1] = ADC_Bemf_CH1;   //BEMF1
  sixStep.Regular_channel[2] = ADC_Bemf_CH2;   //BEMF2
  sixStep.Regular_channel[3] = ADC_Bemf_CH3;   //BEMF3
  sixStep.ADC_SEQ_CHANNEL[0] = ADC_Current;    //CURRENT
  sixStep.ADC_SEQ_CHANNEL[1] = ADC_Pot;        //SPEED
  sixStep.ADC_SEQ_CHANNEL[2] = ADC_Vbus;       //VBUS
  sixStep.ADC_SEQ_CHANNEL[3] = ADC_Temp;       //TEMP

  sixStep.step_position = 0;
  sixStep.demagn_counter = 0;
  sixStep.speed_fdbk_filtered = 0;
  sixStep.Integral_Term_sum = 0;
  sixStep.Current_Reference = 0;
  sixStep.Ramp_Start = 0;
  sixStep.RUN_Motor = false;
  sixStep.speed_fdbk = 0;
  sixStep.ALIGN_OK = false;
  sixStep.VALIDATION_OK = false;
  sixStep.ARR_OK = false;
  sixStep.BEMF_OK = false;
  sixStep.CL_READY = false;
  sixStep.SPEED_VALIDATED = false;
  sixStep.BEMF_Tdown_count = 0;   // Reset of the Counter to detect Stop motor condition when a stall condition occurs

  T_single_step = 0;
  T_single_step_first_value = 0;
  delta = 0;
  Time_vector_tmp = 0;
  Time_vector_prev_tmp = 0;
  Mech_Speed_RPM = 0;
  El_Speed_Hz = 0;
  index_adc_chn = 0;
  mech_accel_hz = 0;
  constant_k = 0;
  ARR_LF = 0;
  index_array = 1;

  Enable_start_button = true;
  startup_bemf_failure = false;
  speed_fdbk_error = false;

  index_ARR_step = 1;
  n_zcr_startup = 0;
  cnt_bemf_event = 0;

  index_align = 1;
  speed_sum_sp_filt = 0;
  speed_sum_pot_filt = 0;
  index_pot_filt = 1;
  potent_filtered = 0;
  Tick_cnt = 0;
  counter_ARR_Bemf = 0;
  constant_multiplier_tmp = 0;

  HFBufferIndex =0;
  for (uint16_t i = 0; i < HFBUFFERSIZE;i++)
    HFBuffer[i]=0;

  for (uint16_t i = 0; i < FILTER_DEEP;i++) {
    speed_tmp_array[i] = 0;
    speed_tmp_buffer[i]= 0;
    }
  array_completed = false;
  buffer_completed = false;

  if (piParam.Reference < 0)
    sixStep.step_position = 1;

  target_speed = TARGET_SPEED;
  setPiParam (&piParam);

  MC_CurrentRefStart();
  MC_CurrentRefSetValue (sixStep.Ireference);

  index_startup_motor = 1;
  rampMotorCalc();
  }
//}}}

//{{{
int32_t MC_GetElSpeedHz() {

  if (__HAL_TIM_GetAutoreload (&hTim6) != 0xFFFF)
    El_Speed_Hz = (int32_t)((sixStep.SYSCLK_frequency) / hTim6.Instance->PSC) / (__HAL_TIM_GetAutoreload (&hTim6) * 6);
  else
    El_Speed_Hz = 0;

  return piParam.Reference < 0 ? -El_Speed_Hz : El_Speed_Hz;
  }
//}}}
//{{{
int32_t MC_GetMechSpeedRPM() {

  Mech_Speed_RPM = (int32_t)(MC_GetElSpeedHz() *  60 / Rotor_poles_pairs);
  return (Mech_Speed_RPM);
  }
//}}}

//{{{
void MC_StartMotor() {

  sixStep.STATUS = START;
  sixStep.RUN_Motor = true;

  HAL_TIM_Base_Start_IT (&hTim6);
  HAL_ADC_Start_IT (&hAdc1);

  NUCLEO_LED_ON();
  }
//}}}
//{{{
void MC_StopMotor() {

  sixStep.STATUS = STOP;
  sixStep.RUN_Motor = false;
  MC_Stop_PWM();

  hTim1.Instance->CR1 &= ~(TIM_CR1_CEN);
  hTim1.Instance->CNT = 0;
  MC_DisableInput_CH1_D_CH2_D_CH3_D();
  HAL_TIM_Base_Stop_IT (&hTim6);
  HAL_ADC_Stop_IT (&hAdc1);

  MC_CurrentRefStop();
  NUCLEO_LED_OFF();
  MC_Reset();
  }
//}}}
//{{{
void MC_Panic() {

  MC_StopMotor();
  sixStep.STATUS = OVERCURRENT;
  }
//}}}

//{{{
void MC_SetSpeed() {

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
void MC_EXTbutton() {

  if (Enable_start_button) {
    if (!sixStep.RUN_Motor && sixStep.Button_ready) {
      printf ("StartMotor\n");
      MC_StartMotor();
      Enable_start_button = false;
      }
    else {
      printf ("StopMotor\n");
      MC_StopMotor();
      Enable_start_button = false;
      }
    }
  }
//}}}

// callbacks
//{{{
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc) {
  MC_ADC();
  }
//}}}
//{{{
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef* htim) {
  MC_Timebase();
  }
//}}}
//{{{
void HAL_SYSTICK_Callback() {
  MC_SysTick();
  }
//}}}
//{{{
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {
  MC_EXTbutton();
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

  MC_Init();

  int loop = 0;
  while (1) {
    HAL_Delay (1000);
    //printf ("loop %d\n", loop++);
    }
  }
//}}}
