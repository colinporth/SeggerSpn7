#pragma once
#include "stdint.h"
#include "stm32f3xx_hal.h"
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

extern ADC_HandleTypeDef hAdc1;
extern ADC_HandleTypeDef hAdc2;
extern ADC_HandleTypeDef hAdc3;
extern TIM_HandleTypeDef hTim1;
extern TIM_HandleTypeDef hTim6;
extern TIM_HandleTypeDef hTim16;

void mcNucleoDisableChan();
void mcNucleoEnableInputChan12();
void mcNucleoEnableInputChan13();
void mcNucleoEnableInputChan23();
void mcNucleoSetChanCCR (uint16_t value1, uint16_t value2, uint16_t value3);
void mcNucleoStartPwm();
void mcNucleoStopPwm();

void mcNucleoCurrentRefStart();
void mcNucleoCurrentRefStop();
void mcNucleoCurrentRefSetValue (uint16_t value);

void mcNucleoAdcChan (ADC_HandleTypeDef* hAdc, uint32_t chan);

void mcNucleoLedOn();
void mcNucleoLedOff();

void mcNucleoInit();

//{{{
#ifdef __cplusplus
}
#endif /* __cplusplus */
//}}}
