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
#include "sixStepLib.h"
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
