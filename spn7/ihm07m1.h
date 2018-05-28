#pragma once
#include "stdint.h"
#include "stm32f3xx_hal.h"
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

//{{{  defines
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
//}}}

//extern ADC_HandleTypeDef hadc1;
//extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim2;
//extern TIM_HandleTypeDef htim6;
//extern TIM_HandleTypeDef htim16;

void MC_ADC_Channel (uint32_t);
void MC_Nucleo_Init();

void MC_EnableInput_CH1_E_CH2_E_CH3_D();
void MC_EnableInput_CH1_E_CH2_D_CH3_E();
void MC_EnableInput_CH1_D_CH2_E_CH3_E();
void MC_DisableInput_CH1_D_CH2_D_CH3_D();
void MC_TIM1_CH1_SetCCR (uint16_t value);
void MC_TIM1_CH2_SetCCR (uint16_t value);
void MC_TIM1_CH3_SetCCR (uint16_t value);
void MC_Start_PWM();
void MC_Stop_PWM();

void MC_CurrentRefStart();
void MC_CurrentRefStop();
void MC_CurrentRefSetValue (uint16_t value);

void NUCLEO_LED_ON();
void NUCLEO_LED_OFF();

//{{{
#ifdef __cplusplus
}
#endif /* __cplusplus */
//}}}
