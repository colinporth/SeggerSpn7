#pragma once
//{{{
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
//}}}

#include "mc_type.h"

#define DOutputActiveHigh       1u
#define DOutputActiveLow        0u

typedef struct
{
	DOutputState_t OutputState;       /*!< indicates the state of the digital output */
	GPIO_TypeDef* hDOutputPort;       /*!< GPIO output port. It must be equal
																			 to GPIOx x= A, B, ...*/
	uint16_t hDOutputPin;             /*!< GPIO output pin. It must be equal to
																			 GPIO_Pin_x x= 0, 1, ...*/
	uint8_t  bDOutputPolarity;        /*!< GPIO output polarity. It must be equal
																			 to DOutputActiveHigh or DOutputActiveLow */
}DOUT_handle_t;

/**
 * @brief  Initializes object variables, port and pin. It must be called only
 *         after PWMnCurrFdbk object initialization and DigitalOutput object
 *         creation.
 * @param pHandle handler address of the digital output component.
 * @retval none.
 */
void DOUT_Init(DOUT_handle_t *pHandle);

/**
 * @brief Accordingly with selected polarity, it sets to active or inactive the
 *        digital output
 * @param pHandle handler address of the digital output component.
 * @param OutputState_t New requested state
 * @retval none
 */
void DOUT_SetOutputState(DOUT_handle_t *pHandle, DOutputState_t State);

/**
 * @brief It returns the state of the digital output
 * @param pHandle pointer on component's handle
 * @retval OutputState_t Digital output state (ACTIVE or INACTIVE)
 */
DOutputState_t DOUT_GetOutputState(DOUT_handle_t *pHandle);

//{{{
#ifdef __cplusplus
}
#endif /* __cpluplus */
//}}}
