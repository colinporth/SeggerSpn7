#include "digital_output.h"
#include "mc_type.h"

/**
* @brief Accordingly with selected polarity, it sets to active or inactive the
*        digital output
* @param pHandle handler address of the digital output component.
* @param OutputState_t New requested state
* @retval none
*/
void DOUT_SetOutputState(DOUT_handle_t *pHandle, DOutputState_t State)
{

  if(State == ACTIVE)
  {
    if(pHandle->bDOutputPolarity == DOutputActiveHigh)
    {
      LL_GPIO_SetOutputPin(pHandle->hDOutputPort,pHandle->hDOutputPin);
    }
    else
    {
      LL_GPIO_ResetOutputPin(pHandle->hDOutputPort,pHandle->hDOutputPin);
    }
  }
  else
    if(pHandle->bDOutputPolarity == DOutputActiveHigh)
    {
      LL_GPIO_ResetOutputPin(pHandle->hDOutputPort,pHandle->hDOutputPin);
    }
    else
    {
      LL_GPIO_SetOutputPin(pHandle->hDOutputPort,pHandle->hDOutputPin);
    }
  pHandle->OutputState = State;
}

/**
* @brief It returns the state of the digital output
* @param pHandle pointer on related component instance
* @retval OutputState_t Digital output state (ACTIVE or INACTIVE)
*/
DOutputState_t DOUT_GetOutputState(DOUT_handle_t *pHandle)
{
  return(pHandle->OutputState);
}
