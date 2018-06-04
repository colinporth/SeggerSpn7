// cSixStep.cpp
//{{{  ihm07m pins
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
#include "../common/cLcd.h"
//}}}

// public
//{{{
void cSixStep::init() {

  mTraceVec.addTrace (1600, 4, 4);

  GPIO_Init();
  ADC_Init();
  TIM1_Init();
  TIM6_Init();
  TIM16_Init();

  // EXTI interrupt init
  HAL_NVIC_SetPriority (EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (EXTI15_10_IRQn);

  // ADC1,2,3 interrupts
  HAL_NVIC_SetPriority (ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (ADC1_2_IRQn);
  HAL_NVIC_SetPriority (ADC3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (ADC3_IRQn);

  // TIM6 interrupts
  HAL_NVIC_SetPriority (TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ (TIM6_DAC_IRQn);

  // sysTick interrupt
  HAL_NVIC_SetPriority (SysTick_IRQn, 2, 0);

  HF_TIMx_ARR = hTim1.Instance->ARR;
  HF_TIMx_PSC = hTim1.Instance->PSC;
  HF_TIMx_CCR = hTim1.Instance->CCR1;

  LF_TIMx_ARR = hTim6.Instance->ARR;
  LF_TIMx_PSC = hTim6.Instance->PSC;

  CW_CCW = TARGET_SPEED < 0;

  reset();
  }
//}}}
//{{{
void cSixStep::reset() {

  mPulseValue = HF_TIMx_CCR;
  mSpeedTargetRamp = MAX_POT_SPEED;
  mDemagnValue = INITIAL_DEMAGN_DELAY;

  hTim1.Init.Prescaler = HF_TIMx_PSC;
  hTim1.Instance->PSC =  HF_TIMx_PSC;
  hTim1.Init.Period =    HF_TIMx_ARR;
  hTim1.Instance->ARR =  HF_TIMx_ARR;
  hTim1.Instance->CCR1 = HF_TIMx_CCR;

  hTim6.Init.Prescaler = LF_TIMx_PSC;
  hTim6.Instance->PSC =  LF_TIMx_PSC;
  hTim6.Init.Period =    LF_TIMx_ARR;
  hTim6.Instance->ARR =  LF_TIMx_ARR;

  mSysClkFrequency = HAL_RCC_GetSysClockFreq();

  ihm07mSetChanCCR (0,0,0);

  mSpeedRef = 0;
  mCurrentReference = 0;
  mBemfDownCount = 0;
  mIntegralTermSum = 0;

  mStartupStepCount = 0;
  mAlignTicks = 1;

  mFirstSingleStep = 0;
  mTimeVectorPrev = 0;
  constant_k = 0;
  mArrLF = 0;

  mOpenLoopBemfEvent = 0;
  mOpenLoopBemfFail = false;
  mSpeedMeasuredFail = false;

  mPrevStep = -1;
  mStep = 5;

  mDemagnCount = 0;
  mRampStepCount = 0;
  mZeroCrossingCount = 0;

  mPot.clear();
  mSpeed.clear();

  mTargetSpeed = TARGET_SPEED;
  setPiParam (&piParam);

  ihm07mCurrentRefStart();
  ihm07mCurrentRefSetValue (mStartupCurrent);

  rampMotor();
  }
//}}}

//{{{
int32_t cSixStep::getSpeed() {

  int32_t speedHz = mSysClkFrequency / (hTim6.Instance->PSC * __HAL_TIM_GetAutoreload (&hTim6) * 6);

  if (piParam.Reference < 0)
    speedHz = -speedHz;

  return (speedHz * 60) / mNumPolePair;
  }
//}}}
//{{{
void cSixStep::setSpeed() {

  if (((mSpeedRef > mSpeedTargetRamp) && (mSpeedRef - mSpeedTargetRamp > ADC_SPEED_TH)) ||
      ((mSpeedRef <= mSpeedTargetRamp) && (mSpeedTargetRamp - mSpeedRef > ADC_SPEED_TH))) {
    mSpeedTargetRamp = mSpeedRef;
    piParam.Reference = CW_CCW ? -mSpeedRef : mSpeedRef;
    }
  }
//}}}

//{{{
void cSixStep::startMotor() {

  mStatus = START;
  HAL_TIM_Base_Start_IT (&hTim6);
  HAL_ADC_Start_IT (&hAdc2);
  HAL_ADC_Start_IT (&hAdc3);

  ihm07mLedOn();
  }
//}}}
//{{{
void cSixStep::stopMotor (eSixStepStatus status) {

  mStatus = status;

  ihm07mStopPwm();
  hTim1.Instance->CR1 &= ~(TIM_CR1_CEN);
  hTim1.Instance->CNT = 0;
  ihm07mDisableChan();

  HAL_TIM_Base_Stop_IT (&hTim6);
  HAL_ADC_Stop_IT (&hAdc2);
  HAL_ADC_Stop_IT (&hAdc3);

  ihm07mCurrentRefStop();
  ihm07mLedOff();

  reset();
  }
//}}}

//{{{
void cSixStep::adcSample (ADC_HandleTypeDef* hadc) {

  if (hadc == &hAdc2) {
    if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) {
      if (mStatus >= STARTUP)
        switch (mStep) {
          //{{{
          case 0: {
            uint16_t value = HAL_ADC_GetValue (hadc);
            if (mDemagnCount < mDemagnValue)
              mDemagnCount++;
            else if (piParam.Reference >= 0) {
              if (value < mBemfDownThreshold)
                arrBemf (false);
              }
            else if (value > mBemfUpThreshold)
              arrBemf (true);

            mTraceVec.addSample (0, value>>4);
            mTraceVec.addSample (1, mStep);
            break;
            }
          //}}}
          //{{{
          case 3: {
            uint16_t value = HAL_ADC_GetValue (hadc);
            if (mDemagnCount >= mDemagnValue)
              mDemagnCount++;
            else if (piParam.Reference >= 0) {
              if (value > mBemfUpThreshold)
                arrBemf (true);
              }
            else if (value < mBemfDownThreshold)
              arrBemf (false);

            mTraceVec.addSample (0, value>>4);
            mTraceVec.addSample (1, mStep);
            break;
            }
          //}}}
          //{{{
          case 2: {
            uint16_t value = HAL_ADC_GetValue (hadc);
            if (mDemagnCount < mDemagnValue)
              mDemagnCount++;
            else if (piParam.Reference >= 0) {
              if (value < mBemfDownThreshold)
                arrBemf (false);
              }
            else if (value > mBemfUpThreshold)
              arrBemf (true);

            mTraceVec.addSample (0, value>>4);
            mTraceVec.addSample (1, mStep);
            break;
            }
          //}}}
          //{{{
          case 5: {
            uint16_t value = HAL_ADC_GetValue (hadc);
            if (mDemagnCount < mDemagnValue)
              mDemagnCount++;
            else if (piParam.Reference >= 0) {
              if (value > mBemfUpThreshold)
                arrBemf (true);
              }
            else if (value < mBemfDownThreshold)
               arrBemf (false);

            mTraceVec.addSample (0, value> 4);
            mTraceVec.addSample (1, mStep);
            break;
            }
          //}}}
          //{{{
          default: {
            uint16_t value = HAL_ADC_GetValue (hadc);
            break;
            }
          //}}}
          }
      // adc2Sample curr or temp
      ihm07mAdcChan (&hAdc2, mAdcIndex ? ADC_CHANNEL_8 : ADC_CHANNEL_7);
      }
    else {
      mAdcValue[mAdcIndex] = HAL_ADC_GetValue (hadc);
      mAdcIndex = (mAdcIndex+1) % 2;
      switch (mStep) {
        case 0:
        case 3:
          ihm07mAdcChan (&hAdc2, ADC_CHANNEL_4); break; // adc2Sample bemf3
        case 2:
        case 5:
          ihm07mAdcChan (&hAdc2, ADC_CHANNEL_9); break; // adc2Sample bemf1
        }
      }
    }

  else { // adc == &hAdc3
    uint16_t value = HAL_ADC_GetValue (hadc);
    if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) {
      if (mStatus >= STARTUP)
        switch (mStep) {
          //{{{
          case 1: {
            uint16_t value = HAL_ADC_GetValue (hadc);
            if (mDemagnCount < mDemagnValue)
              mDemagnCount++;
            else if (piParam.Reference >= 0) {
              if (value > mBemfUpThreshold)
                arrBemf (true);
              }
            else if (value < mBemfDownThreshold)
              arrBemf (false);

            mTraceVec.addSample (0, value>>4);
            mTraceVec.addSample (1, mStep);
            break;
            }
          //}}}
          //{{{
          case 4: {
            uint16_t value = HAL_ADC_GetValue (hadc);
            if (mDemagnCount < mDemagnValue)
              mDemagnCount++;
            else if (piParam.Reference >= 0) {
              if (value < mBemfDownThreshold)
                arrBemf (false);
              }
            else if (value > mBemfUpThreshold)
              arrBemf (true);

            mTraceVec.addSample (0, value>>4);
            mTraceVec.addSample (1, mStep);
            break;
            }
          //}}}
          //{{{
          default: {
            uint16_t value = HAL_ADC_GetValue (hadc);
            break;
            }
          //}}}
          }
      // adc3Sample pot
      ihm07mAdcChan (&hAdc3, ADC_CHANNEL_1);
      }
    else {
      mAdcValue[2] = HAL_ADC_GetValue (hadc);
      // adc3Sample bemf2
      ihm07mAdcChan (&hAdc3, ADC_CHANNEL_12);
      }
    }
  }
//}}}
//{{{
void cSixStep::stepTick() {

  mArrLF = __HAL_TIM_GetAutoreload (&hTim6);

  if (mStatus >= STARTUP) {
    mSpeed.addValue (getSpeed());

    // next step
    if (mStep != mPrevStep)
      mZeroCrossingCount = 0;
    mStep = (piParam.Reference >= 0) ? (mStep + 1) % 6 :  (mStep + 5) % 6;
    mDemagnCount = 1;
    }
  //{{{  set pwm, bemf for step
  switch (mStep) {
    case 0:
      ihm07mSetChanCCR (mPulseValue, 0, 0);
      ihm07mEnableInputChan12();
      if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) // stepChange during downCount, adcSample bemf3
        ihm07mAdcChan (&hAdc2, ADC_CHANNEL_4);
      break;

    case 1:
      ihm07mSetChanCCR (mPulseValue, 0, 0);
      ihm07mEnableInputChan13();
      if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) // stepChange during downCount, adcSample bemf2
        ihm07mAdcChan (&hAdc3, ADC_CHANNEL_12);
      break;

    case 2:
      ihm07mSetChanCCR (0, mPulseValue, 0);
      ihm07mEnableInputChan23();
      if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) // stepChange during downCount, adcSample bemf1
        ihm07mAdcChan (&hAdc2, ADC_CHANNEL_9);
      break;

    case 3:
      ihm07mSetChanCCR (0, mPulseValue, 0);
      ihm07mEnableInputChan12();
      if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) // stepChange during downCount, adcSample bemf3
        ihm07mAdcChan (&hAdc2, ADC_CHANNEL_4);
     break;

    case 4:
      ihm07mSetChanCCR (0, 0, mPulseValue);
      ihm07mEnableInputChan13();
      if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) // stepChange during downCount, adcSample bemf2
        ihm07mAdcChan (&hAdc3, ADC_CHANNEL_12);
      break;

    case 5:
      ihm07mSetChanCCR (0, 0, mPulseValue);
      ihm07mEnableInputChan23();
      if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) // stepChange during downCount, adcSample bemf1
        ihm07mAdcChan (&hAdc2, ADC_CHANNEL_9);
      break;
     }
  //}}}

  mTraceVec.addSample (2, getSpeedFiltered()>>4);
  mTraceVec.addSample (3, mPot.getFilteredValue()>>4);

  if (mStatus == STARTUP) {
    rampMotor();
    if (mRampStepCount < mMaxNumRampSteps) {
      hTim6.Init.Period = mArrValue;
      hTim6.Instance->ARR = (uint32_t)hTim6.Init.Period;
      mRampStepCount++;
      }
    }
  else if (mStatus >= SPEED_OK) {
    // motorStall detection
    if (mBemfDownCount++ > BEMF_CONSEC_DOWN_MAX)
      mSpeedMeasuredFail = true;
    else
      __HAL_TIM_SetAutoreload (&hTim6, 0xFFFF);
    }
  }
//}}}
//{{{
void cSixStep::sysTick() {

  if (mStatus == START) {
    //{{{  start alignment
    mStatus = ALIGN;
    mStep = 5;

    hTim6.Init.Period = mArrValue;
    hTim6.Instance->ARR = (uint32_t)hTim6.Init.Period;
    ihm07mStartPwm();
    }
    //}}}

  if (mStatus == ALIGN) {
    mAlignTicks++;
    if (mAlignTicks > ALIGN_MS) {
      //{{{  align time elapsed
      hTim6.Init.Prescaler = prescaler_value;
      hTim6.Instance->PSC = hTim6.Init.Prescaler;
      mStatus = STARTUP;
      }
      //}}}
    mTargetSpeed = MIN_POT_SPEED + (mAdcValue[2] * (MAX_POT_SPEED - MIN_POT_SPEED)) / 4096;
    }

  mPot.addValue (mAdcValue[2]);
  mSpeedRef = MIN_POT_SPEED + (((MAX_POT_SPEED - MIN_POT_SPEED) * mPot.getFilteredValue()) / 4096);

  if (mStatus != SPEED_FEEDBACK_FAIL) {
    int16_t speedFiltered = getSpeedFiltered();

    if ((mStatus == STARTUP) && ((speedFiltered > mTargetSpeed) || (speedFiltered < -mTargetSpeed)))
      mStatus = SPEED_OK;
    if (mStatus == BEMF_OK)
      mStatus = RUN;

    if (mStatus == RUN) {
      uint16_t ref = (uint16_t)piController (&piParam, speedFiltered);
      if (piParam.Reference < 0)
        ref = -ref;
      mCurrentReference = ref;
      ihm07mCurrentRefSetValue (mCurrentReference);
      }
    mDemagnValue = getDemagnValue (piParam.Reference, speedFiltered);
    }

  if (mStatus >= SPEED_OK)
    setSpeed();

  if (mOpenLoopBemfFail) {
    ACCEL >>= 1;
    if (ACCEL < MINIMUM_ACC)
      ACCEL = MINIMUM_ACC;

    stopMotor (STARTUP_BEMF_FAIL);
    mOpenLoopBemfEvent = 0;
    }

  if (mSpeedMeasuredFail)
    stopMotor (SPEED_FEEDBACK_FAIL);
  }
//}}}

// private
//{{{
void cSixStep::GPIO_Init() {
// config
// PB2  -> redLed
// PC13 <- blueButton + extInt

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // config gpio pin PB2 - output ihm07m redLed
  GPIO_InitTypeDef gpioInit;
  gpioInit.Pin = GPIO_PIN_2;
  gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
  gpioInit.Pull = GPIO_NOPULL;
  gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOB, &gpioInit);
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  // config gpio PC13 input interrupt - ihm07mF303re blueButton
  gpioInit.Pin = GPIO_PIN_13;
  gpioInit.Mode = GPIO_MODE_IT_RISING;
  gpioInit.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOC, &gpioInit);
  }
//}}}
//{{{
void cSixStep::ADC_Init() {
//   PC3  -> ADC12_IN9  bemf1
//   PB0  -> ADC3_IN12  bemf2
//   PA7  -> ADC2_IN4   bemf3

//   PC1  -> ADC12_IN7  currFdbk2 - 1shunt, 3shunt
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
  ADC_ChannelConfTypeDef channelConfig;
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
  channelConfig.Channel = ADC_CHANNEL_1;
  channelConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;

  if (HAL_ADC_ConfigChannel (&hAdc3, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel3 failed\n");
  //}}}
  //{{{  config PB0 bemf2 - adc3  chan 12
  channelConfig.Channel = ADC_CHANNEL_12;
  channelConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;

  if (HAL_ADC_ConfigChannel (&hAdc3, &channelConfig) != HAL_OK)
    printf ("HAL_ADC_ConfigChannel3 failed\n");
  //}}}
  }
//}}}
//{{{
void cSixStep::TIM1_Init() {
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
void cSixStep::TIM6_Init() {
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
  }
//}}}
//{{{
void cSixStep::TIM16_Init() {
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
void cSixStep::ihm07mDisableChan() {

  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);  // EN1 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);  // EN2 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);  // EN3 DISABLE
  }
//}}}
//{{{
void cSixStep::ihm07mEnableInputChan12() {

  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_10, GPIO_PIN_SET);    // EN1 ENABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_11, GPIO_PIN_SET);    // EN2 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);  // EN3 ENABLE
  }
//}}}
//{{{
void cSixStep::ihm07mEnableInputChan13() {

  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_10, GPIO_PIN_SET);   // EN1 ENABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // EN2 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_12, GPIO_PIN_SET);   // EN3 ENABLE
  }
//}}}
//{{{
void cSixStep::ihm07mEnableInputChan23() {

  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // EN1 DISABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_11, GPIO_PIN_SET);   // EN2 ENABLE
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_12, GPIO_PIN_SET);   // EN3 ENABLE
  }
//}}}
//{{{
void cSixStep::ihm07mSetChanCCR (uint16_t value1, uint16_t value2, uint16_t value3) {
  hTim1.Instance->CCR1 = value1;
  hTim1.Instance->CCR2 = value2;
  hTim1.Instance->CCR3 = value3;
  }
//}}}
//{{{
void cSixStep::ihm07mStartPwm() {
  HAL_TIM_PWM_Start (&hTim1, TIM_CHANNEL_1);  // TIM1_CH1 ENABLE
  HAL_TIM_PWM_Start (&hTim1, TIM_CHANNEL_2);  // TIM1_CH2 ENABLE
  HAL_TIM_PWM_Start (&hTim1, TIM_CHANNEL_3);  // TIM1_CH3 ENABLE
  }
//}}}
//{{{
void cSixStep::ihm07mStopPwm() {
  HAL_TIM_PWM_Stop (&hTim1, TIM_CHANNEL_1);  // TIM1_CH1 DISABLE
  HAL_TIM_PWM_Stop (&hTim1, TIM_CHANNEL_2);  // TIM1_CH2 DISABLE
  HAL_TIM_PWM_Stop (&hTim1, TIM_CHANNEL_3);  // TIM1_CH3 DISABLE
  }
//}}}

//{{{
void cSixStep::ihm07mCurrentRefStart() {

  hTim16.Instance->CCR1 = 0;
  HAL_TIM_PWM_Start (&hTim16, TIM_CHANNEL_1);
  }
//}}}
//{{{
void cSixStep::ihm07mCurrentRefStop() {

  hTim16.Instance->CCR1 = 0;
  HAL_TIM_PWM_Stop (&hTim16, TIM_CHANNEL_1);
  }
//}}}
//{{{
void cSixStep::ihm07mCurrentRefSetValue (uint16_t value) {
  hTim16.Instance->CCR1 = (uint32_t)(value * hTim16.Instance->ARR) / 4096;
  }
//}}}

//{{{
void cSixStep::ihm07mAdcChan (ADC_HandleTypeDef* adc, uint32_t chan) {

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
void cSixStep::ihm07mLedOn() {
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  }
//}}}
//{{{
void cSixStep::ihm07mLedOff() {
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  }
//}}}

//{{{
uint64_t cSixStep::fastSqrt (uint64_t input) {

  const uint16_t kShiftnsqrt = 14;

  uint64_t root;
  if (input <= (uint64_t)((uint64_t)2097152 << kShiftnsqrt))
    root = (uint64_t)((uint64_t)128 << kShiftnsqrt);
  else
    root = (uint64_t)((uint64_t)8192 << kShiftnsqrt);

  uint8_t biter = 0u;
  uint64_t rootNew;
  do {
    rootNew = (root + input / root) >> 1;
    if (rootNew == root)
      biter = kShiftnsqrt - 1;
    else {
      biter++;
      root = rootNew;
      }
    } while (biter < kShiftnsqrt-1);

  return rootNew;
  }
//}}}

//{{{
void cSixStep::rampMotor() {

  if (mStartupStepCount == 0) {
    uint32_t mech_accel_hz = ACCEL * mNumPolePair / 60;
    constant_k = ((uint64_t)100 * (uint64_t)4000000000) / (3 * mech_accel_hz);
    ihm07mCurrentRefSetValue (mStartupCurrent);
    mTimeVectorPrev = 0;
    }

  uint32_t timeVector = 0;
  if (mStartupStepCount < MAX_STARTUP_STEPS) {
    timeVector = ((uint64_t)1000000 * (uint64_t)fastSqrt(((uint64_t)(mStartupStepCount+1) * (uint64_t)constant_k))) / 632455;
    int32_t delta = timeVector - mTimeVectorPrev;
    if (mStartupStepCount == 0) {
      mFirstSingleStep = (2 * 3141 * delta) / 1000;
      mArrValue = (uint32_t)(0xFFFF);
      }
    else {
      uint32_t singleStep = (2 * 3141* delta) / 1000;
      mArrValue = (uint32_t)(0xFFFF * singleStep) / mFirstSingleStep;
      }
    }
  else
    mStartupStepCount = 0;

  if (mStartupStepCount == 0)
    prescaler_value = (((mSysClkFrequency / 1000000) * mFirstSingleStep) / 0xFFFF) - 1;

  if (mStatus == STARTUP)
    mStartupStepCount++;
  else
    timeVector = 0;

  mTimeVectorPrev = timeVector;
  }
//}}}
//{{{
void cSixStep::arrBemf (bool up) {

  if (up)
    mBemfDownCount = 0;

  if (mStep != mPrevStep) {
    mPrevStep = mStep;

    if (mStatus == SPEED_OK) {
      if (mOpenLoopBemfEvent > BEMF_CNT_EVENT_MAX)
        mOpenLoopBemfFail = true;

      if (up) {
        mZeroCrossingCount++;
        mOpenLoopBemfEvent = 0;
        }
      else
        mOpenLoopBemfEvent++;

      if (mZeroCrossingCount >= MAX_STARTUP_ZERO_CROSSING) {
        mStatus = BEMF_OK;
        mZeroCrossingCount = 0;
        }
      }

    if (mStatus >= SPEED_OK)
      __HAL_TIM_SetAutoreload (&hTim6, __HAL_TIM_GetCounter (&hTim6) + mArrLF/2);
    }
  }
//}}}

//{{{
void cSixStep::setPiParam (cPiParam* piParam) {

  piParam->Reference = CW_CCW ? -mTargetSpeed : mTargetSpeed;

  piParam->Kp_Gain = KP;
  piParam->Ki_Gain = KI;

  piParam->Lower_Limit_Output = LOWER_OUT_LIMIT;
  piParam->Upper_Limit_Output = UPPER_OUT_LIMIT;

  piParam->Max_PID_Output = false;
  piParam->Min_PID_Output = false;
  }
//}}}
//{{{
int16_t cSixStep::piController (cPiParam* PI_PARAM, int16_t speed_fdb) {

  int32_t error = PI_PARAM->Reference - speed_fdb;

  // Proportional term computation
  int32_t wProportional_Term = PI_PARAM->Kp_Gain * error;

  // Integral term computation
  int32_t wIntegral_Term = 0;
  int32_t wIntegral_sum_temp = 0;
  if (PI_PARAM->Ki_Gain == 0)
    mIntegralTermSum = 0;
  else {
    wIntegral_Term = PI_PARAM->Ki_Gain * error;
    wIntegral_sum_temp = mIntegralTermSum + wIntegral_Term;
    mIntegralTermSum = wIntegral_sum_temp;
    }

  if (mIntegralTermSum > KI_DIV * PI_PARAM->Upper_Limit_Output)
    mIntegralTermSum = KI_DIV * PI_PARAM->Upper_Limit_Output;

  if (mIntegralTermSum < -KI_DIV * PI_PARAM->Upper_Limit_Output)
    mIntegralTermSum = -KI_DIV * PI_PARAM->Upper_Limit_Output;

  // WARNING: the below instruction is not MISRA compliant, user should verify
  //          that Cortex-M3 assembly instruction ASR (arithmetic shift right)
  //          is used by the compiler to perform the shifts (instead of LSR logical shift right)
  int32_t wOutput_32 = (wProportional_Term / KP_DIV) + (mIntegralTermSum / KI_DIV);

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
uint16_t cSixStep::getDemagnValue (uint16_t piReference, int16_t speed) {

  if (piReference >= 0) {
    if (speed > 10000)
      return DEMAGN_VAL_1;
    if (speed > 7800)
      return DEMAGN_VAL_2;
    if (speed > 6400)
      return DEMAGN_VAL_3;
    if (speed > 5400)
      return DEMAGN_VAL_4;
    if (speed > 4650)
      return DEMAGN_VAL_5;
    if (speed > 4100)
      return DEMAGN_VAL_6;
    if (speed > 3650)
      return DEMAGN_VAL_7;
    if (speed > 3300)
      return DEMAGN_VAL_8;
    if (speed > 2600)
      return DEMAGN_VAL_9;
    if (speed > 1800)
      return DEMAGN_VAL_10;
    if (speed > 1500)
      return DEMAGN_VAL_11;
    if (speed > 1300)
      return DEMAGN_VAL_12;
    if (speed > 1000)
      return DEMAGN_VAL_13;
    return DEMAGN_VAL_14;
    }
  else {
    if (speed < -10000)
      return DEMAGN_VAL_1;
    if (speed < -7800)
      return DEMAGN_VAL_2;
    if (speed < -6400)
      return DEMAGN_VAL_3;
    if (speed < -5400)
      return DEMAGN_VAL_4;
    if (speed < -4650)
      return DEMAGN_VAL_5;
    if (speed < -4100)
      return DEMAGN_VAL_6;
    if (speed < -3650)
      return DEMAGN_VAL_7;
    if (speed < -3300)
      return DEMAGN_VAL_8;
    if (speed < -2650)
      return DEMAGN_VAL_9;
    if (speed <-1800)
      return DEMAGN_VAL_10;
    if (speed < -1500)
      return DEMAGN_VAL_11;
    if (speed < -1300)
      return DEMAGN_VAL_12;
    if (speed < -1000)
      return DEMAGN_VAL_13;
    return DEMAGN_VAL_14;
    }
  }
//}}}
