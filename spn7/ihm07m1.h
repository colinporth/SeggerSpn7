#pragma once
#include "stdint.h"
#include "stm32f3xx_hal.h"
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

// defines
#define ADC_Bemf_CH1       ADC_CHANNEL_9   // BEMF1
#define ADC_Bemf_CH2       ADC_CHANNEL_11  // BEMF2
#define ADC_Bemf_CH3       ADC_CHANNEL_15  // BEMF3

#define ADC_Current        ADC_CHANNEL_7   // current
#define ADC_Pot            ADC_CHANNEL_12  // pot
#define ADC_Vbus           ADC_CHANNEL_2   // vbus
#define ADC_Temp           ADC_CHANNEL_8   // temp

#define DAC_ENABLE         1               // Enable DAC peripheral
#define DACx               hdac
#define DACx_CH            DAC1_CHANNEL_1  // DAC Channel
#define DACx_ALIGN         DAC_ALIGN_12B_L // DAC Aligment value

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
