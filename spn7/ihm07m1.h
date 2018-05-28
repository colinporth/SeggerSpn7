#pragma once
#include "stdint.h"
#include "stm32f3xx_hal.h"
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

void mcAdcChannel (uint32_t channel);
void mcNucleo_Init();

void mcEnableInput_CH1_E_CH2_E_CH3_D();
void mcEnableInput_CH1_E_CH2_D_CH3_E();
void mcEnableInput_CH1_D_CH2_E_CH3_E();
void mcDisableInput_CH1_D_CH2_D_CH3_D();
void mcTIM1_CH1_SetCCR (uint16_t value);
void mcTIM1_CH2_SetCCR (uint16_t value);
void mcTIM1_CH3_SetCCR (uint16_t value);
void mcStart_PWM();
void mcStop_PWM();

void mcCurrentRefStart();
void mcCurrentRefStop();
void mcCurrentRefSetValue (uint16_t value);

void mcNucleo_Led_On();
void mcNucleo_Led_Off();

//{{{
#ifdef __cplusplus
}
#endif /* __cplusplus */
//}}}
