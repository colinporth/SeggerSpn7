#pragma once
#include "stdint.h"
#include "stm32f3xx_hal.h"
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

//{{{  defines
#define HF_TIMx            htim1
#define HALL_ENCODER_TIMx  htim2
#define LF_TIMx            htim6
#define REFx               htim16
#define ADCx               hadc1

#define HF_TIMx_CH1        TIM_CHANNEL_1
#define HF_TIMx_CH2        TIM_CHANNEL_2
#define HF_TIMx_CH3        TIM_CHANNEL_3
#define HF_TIMx_CCR1       CCR1            // Channel 1
#define HF_TIMx_CCR2       CCR2            // Channel 2
#define HF_TIMx_CCR3       CCR3            // Channel 3

#define ADC_Bemf_CH1       ADC_CHANNEL_9   // BEMF1
#define ADC_Bemf_CH2       ADC_CHANNEL_11  // BEMF2
#define ADC_Bemf_CH3       ADC_CHANNEL_15  // BEMF3

#define ADC_CH_1           ADC_CHANNEL_7   // CURRENT
#define ADC_CH_2           ADC_CHANNEL_12  // SPEED
#define ADC_CH_3           ADC_CHANNEL_2   // VBUS
#define ADC_CH_4           ADC_CHANNEL_8   // TEMP
#define DAC_ENABLE         1               // Enable DAC peripheral
#define DACx               hdac
#define DACx_CH            DAC1_CHANNEL_1  // DAC Channel
#define DACx_ALIGN         DAC_ALIGN_12B_L // DAC Aligment value
//}}}

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim16;

void MC_BemfDelayCalc();

void MC_ADC_Channel (uint32_t);
void MC_Nucleo_Init();

void MC_EnableInput_CH1_E_CH2_E_CH3_D();
void MC_EnableInput_CH1_E_CH2_D_CH3_E();
void MC_EnableInput_CH1_D_CH2_E_CH3_E();
void MC_DisableInput_CH1_D_CH2_D_CH3_D();

void MC_Start_PWM();
void MC_Stop_PWM();

void MC_HF_TIMx_SetDutyCycle_CH1 (uint16_t value);
void MC_HF_TIMx_SetDutyCycle_CH2 (uint16_t value);
void MC_HF_TIMx_SetDutyCycle_CH3 (uint16_t value);

void MC_Current_Reference_Start();
void MC_Current_Reference_Stop();
void MC_Current_Reference_Setvalue (uint16_t value);

void NUCLEO_LED_ON();
void NUCLEO_LED_OFF();

//{{{
#ifdef __cplusplus
}
#endif /* __cplusplus */
//}}}
