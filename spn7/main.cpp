// main.c
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
// PC1 -> ADC12_IN7  curr_fdbk2 - 1shunt
// uPA1 -> ADC1_IN2   vbus
// PC2 -> ADC12_IN8  temp
// PB1 -> ADC3_IN1   pot
//
// PC3 -> ADC12_IN9  bemf1
// PB0 -> ADC3_IN12  bemf2
// PA7 -> ADC2_IN4   bemf3

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

#include "cSixStep.h"
#include "cLcd.h"
//}}}
//{{{  const
const char* kStatusStr[12] = {
  "startupBemf FAIL",
  "overcurrent FAIL",
  "speed feedback FAIL",
  "startup FAIL",
  "init",
  "stopped",
  "start",
  "align",
  "startup",
  "speedOk",
  "bemfOk",
  "run" };
//}}}

cSixStep gSixStep;
uint32_t gLastButtonPress = 0;

//{{{
extern "C" {
  void ADC1_2_IRQHandler() { HAL_ADC_IRQHandler (&gSixStep.hAdc2); }
  void ADC3_IRQHandler()   { HAL_ADC_IRQHandler (&gSixStep.hAdc3); }

  void TIM6_DAC_IRQHandler() { HAL_TIM_IRQHandler (&gSixStep.hTim6); }
  void EXTI15_10_IRQHandler() { HAL_GPIO_EXTI_IRQHandler (GPIO_PIN_13); }
  //{{{
  void TIM1_BRK_TIM15_IRQHandler() {

    if (__HAL_TIM_GET_FLAG (&gSixStep.hTim1, TIM_FLAG_BREAK) != RESET)
      gSixStep.stopMotor (cSixStep::OVERCURRENT_FAIL);

    HAL_TIM_IRQHandler (&gSixStep.hTim1);
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

// callbacks
//{{{
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc) {
  gSixStep.adcSample (hadc);
  }
//}}}
//{{{
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef* htim) {
  if (htim == &gSixStep.hTim6)
    gSixStep.tim6Tick();
  }
//}}}
//{{{
void HAL_SYSTICK_Callback() {
  gSixStep.sysTick();
  }
//}}}
//{{{
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {
// blue button release

  if (HAL_GetTick() > gLastButtonPress + 200) {
    gLastButtonPress = HAL_GetTick();
    if (gSixStep.mStatus < cSixStep::START)
      gSixStep.startMotor();
    else
      gSixStep.stopMotor (cSixStep::STOPPED);
    }
  }
//}}}

//{{{
void systemClockConfig() {

  // config osc
  RCC_OscInitTypeDef rccOscInitStruct;
  rccOscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  rccOscInitStruct.HSEState = RCC_HSE_ON;
  rccOscInitStruct.HSIState = RCC_HSI_ON;
  rccOscInitStruct.PLL.PLLState = RCC_PLL_ON;
  rccOscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  rccOscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  rccOscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig (&rccOscInitStruct) != HAL_OK)
    printf ("HAL_RCC_OscConfig failed\n");

  // config CPU, AHB and APB clocks
  RCC_ClkInitTypeDef rccClkInitStruct;
  rccClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                               RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  rccClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  rccClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  rccClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  rccClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig (&rccClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    printf ("HAL_RCC_ClockConfig failed\n");

  // config periph clocks
  RCC_PeriphCLKInitTypeDef periphClkInit;
  periphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_ADC34 |
                                       RCC_PERIPHCLK_TIM1  | RCC_PERIPHCLK_TIM16;
  periphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  periphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  periphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  periphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig (&periphClkInit) != HAL_OK)
    printf ("HAL_RCCEx_PeriphCLKConfig failed\n");

  // enable Clock Security System
  HAL_RCC_EnableCSS();

  // config Systick interrupt
  HAL_SYSTICK_Config (HAL_RCC_GetHCLKFreq() / 1000);
  HAL_SYSTICK_CLKSourceConfig (SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority (SysTick_IRQn, 4, 0);
  }
//}}}
//{{{
int main() {

  HAL_Init();
  systemClockConfig();

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping (NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority (MemoryManagement_IRQn, 0, 0);
  HAL_NVIC_SetPriority (BusFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority (UsageFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority (SVCall_IRQn, 0, 0);
  HAL_NVIC_SetPriority (DebugMonitor_IRQn, 0, 0);
  HAL_NVIC_SetPriority (PendSV_IRQn, 0, 0);

  gSixStep.init();

  cLcd lcd;
  lcd.init();

  while (true) {
    lcd.clear (cLcd::eOn);
    lcd.drawString (cLcd::eOff, cLcd::eBig, cLcd::eLeft,
                    "c:" + dec (gSixStep.mAdcValue[0], 4) +
                    " t:" + dec (gSixStep.mAdcValue[1], 4) +
                    " p:" + dec (gSixStep.mAdcValue[2], 4),
                    cPoint(0,0));
    lcd.drawString (cLcd::eOff, cLcd::eBig, cLcd::eLeft,
                    std::string(kStatusStr[gSixStep.mStatus]) + " " +
                    dec (gSixStep.getSpeedFiltered(), 4) + " " + dec (gSixStep.mSpeedRef,4),
                    cPoint(0,40));
    gSixStep.getTraceVec()->draw (&lcd, 80, lcd.getHeight());
    lcd.present();
    }
  }
//}}}
