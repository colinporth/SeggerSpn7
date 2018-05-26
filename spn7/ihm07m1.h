#pragma once
#include "stdint.h"
#include "stm32f3xx_hal.h"
//{{{
#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//}}}

//{{{  defines
#define HF_TIMx            htim1
#define HALL_ENCODER_TIMx  htim2
#define LF_TIMx            htim6
#define REFx               htim16
#define ADCx               hadc1
#define UART               huart2

#define GPIO_PORT_1        GPIOC
#define GPIO_CH1           GPIO_PIN_10
#define GPIO_PORT_2        GPIOC
#define GPIO_CH2           GPIO_PIN_11
#define GPIO_PORT_3        GPIOC
#define GPIO_CH3           GPIO_PIN_12
#define GPIO_SET           GPIO_PIN_SET
#define GPIO_RESET         GPIO_PIN_RESET

#define ADC_CH_1           ADC_CHANNEL_7    /*CURRENT*/
#define ADC_CH_2           ADC_CHANNEL_12   /*SPEED*/
#define ADC_CH_3           ADC_CHANNEL_2    /*VBUS*/
#define ADC_CH_4           ADC_CHANNEL_8    /*TEMP*/
#define ADC_Bemf_CH1       ADC_CHANNEL_9    /*BEMF1*/
#define ADC_Bemf_CH2       ADC_CHANNEL_11   /*BEMF2*/
#define ADC_Bemf_CH3       ADC_CHANNEL_15   /*BEMF3*/

#define ADC_CH_1_ST        ADC_SAMPLETIME_1CYCLE_5    /*CURRENT sampling time */
#define ADC_CH_2_ST        ADC_SAMPLETIME_181CYCLES_5 /*SPEED sampling time*/
#define ADC_CH_3_ST        ADC_SAMPLETIME_181CYCLES_5 /*VBUS sampling time*/
#define ADC_CH_4_ST        ADC_SAMPLETIME_181CYCLES_5 /*TEMP sampling time*/
#define ADC_Bemf_CH1_ST    ADC_SAMPLETIME_61CYCLES_5  /*BEMF1 sampling time*/
#define ADC_Bemf_CH2_ST    ADC_SAMPLETIME_61CYCLES_5  /*BEMF2 sampling time*/
#define ADC_Bemf_CH3_ST    ADC_SAMPLETIME_61CYCLES_5  /*BEMF3 sampling time*/

#define HF_TIMx_CH1        TIM_CHANNEL_1
#define HF_TIMx_CH2        TIM_CHANNEL_2
#define HF_TIMx_CH3        TIM_CHANNEL_3
#define HF_TIMx_CCR1       CCR1            /*Channel 1*/
#define HF_TIMx_CCR2       CCR2            /*Channel 2*/
#define HF_TIMx_CCR3       CCR3            /*Channel 3*/

#define DAC_ENABLE         1               /*!< Enable (1) the DAC peripheral */
#define DACx               hdac
#define DACx_CH            DAC1_CHANNEL_1  /*!<  DAC Channel */
#define DACx_ALIGN         DAC_ALIGN_12B_L  /*!< DAC Aligment value */

#define GPIO_PORT_ZCR      GPIOC           /*!<  GPIO port name for zero crossing detection */
#define GPIO_CH_ZCR        GPIO_PIN_7      /*!<  GPIO pin name for zero crossing detection */
#define GPIO_PORT_COMM     GPIOC           /*!<  GPIO port name for 6Step commutation */
#define GPIO_CH_COMM       GPIO_PIN_4      /*!<  GPIO pin name for 6Step commutation */

#define STARTM_CMD  0     /*!<  Start Motor command received */
#define STOPMT_CMD  1     /*!<  Stop Motor command received */
#define SETSPD_CMD  2     /*!<  Set the new speed value command received */
#define GETSPD_CMD  3     /*!<  Get Mechanical Motor Speed command received */
#define INIREF_CMD  4     /*!<  Set the new STARUP_CURRENT_REFERENCE value command received */
#define POLESP_CMD  5     /*!<  Set the Pole Pairs value command received */
#define ACCELE_CMD  6     /*!<  Set the Accelleration for Start-up of the motor command received */
#define KP_PRM_CMD  7     /*!<  Set the KP PI param command received */
#define KI_PRM_CMD  8     /*!<  Set the KI PI param command received */
#define POTENZ_CMD  9     /*!<  Enable Potentiometer command received */
#define HELP_CMD    10    /*!<  Help command received */
#define STATUS_CMD  11    /*!<  Get the Status of the system command received */
#define DIRECT_CMD  12    /*!<  Get the motor direction */
//}}}

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim16;
extern DAC_HandleTypeDef hdac;
extern UART_HandleTypeDef huart2;

void START_Ref_Generation();
void STOP_Ref_Generation();
void Set_Ref_Generation (uint16_t);

void START_DAC();
void STOP_DAC();
void SET_DAC_value (uint16_t);

void bemfDelayCalc();

void MC_SixStep_ADC_Channel (uint32_t);
void MC_SixStep_Nucleo_Init();

void MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D();
void MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E();
void MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E();
void MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D();

void MC_SixStep_Start_PWM_driving();
void MC_SixStep_Stop_PWM_driving();

void MC_SixStep_HF_TIMx_SetDutyCycle_CH1 (uint16_t);
void MC_SixStep_HF_TIMx_SetDutyCycle_CH2 (uint16_t);
void MC_SixStep_HF_TIMx_SetDutyCycle_CH3 (uint16_t);

void MC_SixStep_Current_Reference_Start();
void MC_SixStep_Current_Reference_Stop();
void MC_SixStep_Current_Reference_Setvalue (uint16_t);

void BSP_X_NUCLEO_FAULT_LED_ON();
void BSP_X_NUCLEO_FAULT_LED_OFF();

//{{{
#ifdef __cplusplus
}
#endif /* __cplusplus */
//}}}
