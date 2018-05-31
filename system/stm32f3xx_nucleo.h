#pragma once
//{{{
#ifdef __cplusplus
  extern "C" {
#endif
//}}}
#include "stm32f3xx_hal.h"

#define USE_STM32F3XX_NUCLEO

typedef enum { LED2 = 0, LED_GREEN = LED2 } Led_TypeDef;
typedef enum { BUTTON_USER = 0, /* Alias */ BUTTON_KEY  = BUTTON_USER } Button_TypeDef;
typedef enum { BUTTON_MODE_GPIO = 0, BUTTON_MODE_EXTI = 1 } ButtonMode_TypeDef;

#define LEDn     1
#define BUTTONn  1

void BSP_LED_Init (Led_TypeDef Led);
void BSP_LED_DeInit (Led_TypeDef Led);
void BSP_LED_On (Led_TypeDef Led);
void BSP_LED_Off (Led_TypeDef Led);
void BSP_LED_Toggle (Led_TypeDef Led);

void BSP_PB_Init (Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void BSP_PB_DeInit (Button_TypeDef Button);
uint32_t BSP_PB_GetState (Button_TypeDef Button);

void SPIx_Init();
void SPIx_WriteReadData (const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth);
uint32_t SPIx_Read();
void SPIx_Write (uint8_t Value);

//{{{
#ifdef __cplusplus
}
#endif
//}}}
