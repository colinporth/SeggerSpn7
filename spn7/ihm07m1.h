#pragma once
#include "stdint.h"
#include "stm32f3xx_hal.h"
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

void mcNucleoAdcChannel (uint32_t channel);
void mcNucleoNucleo_Init();

void mcNucleoEnableInput_CH1_E_CH2_E_CH3_D();
void mcNucleoEnableInput_CH1_E_CH2_D_CH3_E();
void mcNucleoEnableInput_CH1_D_CH2_E_CH3_E();
void mcNucleoDisableInput_CH1_D_CH2_D_CH3_D();
void mcNucleoTIM1_CH1_SetCCR (uint16_t value);
void mcNucleoTIM1_CH2_SetCCR (uint16_t value);
void mcNucleoTIM1_CH3_SetCCR (uint16_t value);
void mcNucleoStart_PWM();
void mcNucleoStop_PWM();

void mcNucleoCurrentRefStart();
void mcNucleoCurrentRefStop();
void mcNucleoCurrentRefSetValue (uint16_t value);

void mcNucleoLedOn();
void mcNucleoLedOff();

//{{{
#ifdef __cplusplus
}
#endif /* __cplusplus */
//}}}
