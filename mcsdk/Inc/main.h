#pragma once

#define Start_Stop_Pin GPIO_PIN_13
#define Start_Stop_GPIO_Port GPIOC
#define M1_CURR_AMPL_W_Pin GPIO_PIN_0
#define M1_CURR_AMPL_W_GPIO_Port GPIOC
#define M1_CURR_AMPL_V_Pin GPIO_PIN_1
#define M1_CURR_AMPL_V_GPIO_Port GPIOC
#define M1_TEMPERATURE_Pin GPIO_PIN_2
#define M1_TEMPERATURE_GPIO_Port GPIOC
#define M1_CURR_AMPL_U_Pin GPIO_PIN_0
#define M1_CURR_AMPL_U_GPIO_Port GPIOA
#define M1_BUS_VOLTAGE_Pin GPIO_PIN_1
#define M1_BUS_VOLTAGE_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_2
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_3
#define UART_RX_GPIO_Port GPIOA
#define DBG_DAC_CH1_Pin GPIO_PIN_4
#define DBG_DAC_CH1_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define M1_PWM_UH_Pin GPIO_PIN_8
#define M1_PWM_UH_GPIO_Port GPIOA
#define M1_PWM_VH_Pin GPIO_PIN_9
#define M1_PWM_VH_GPIO_Port GPIOA
#define M1_PWM_WH_Pin GPIO_PIN_10
#define M1_PWM_WH_GPIO_Port GPIOA
#define M1_OCP_Pin GPIO_PIN_11
#define M1_OCP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define M1_PWM_EN_U_Pin GPIO_PIN_10
#define M1_PWM_EN_U_GPIO_Port GPIOC
#define M1_PWM_EN_V_Pin GPIO_PIN_11
#define M1_PWM_EN_V_GPIO_Port GPIOC
#define M1_PWM_EN_W_Pin GPIO_PIN_12
#define M1_PWM_EN_W_GPIO_Port GPIOC

//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}
void _Error_Handler(char *, int);
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
//{{{
#ifdef __cplusplus
}
#endif
//}}}
