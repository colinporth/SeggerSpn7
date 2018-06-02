#include "user_interface.h"
#include "dac_common_ui.h"

/** @defgroup dac_rctimer_ui DAC RC Timer User Interface
  *
  * @brief RC Timer DAC User Interface implementation
  *
  * Digital to Analog Converter common component. In Motor Control context, the DAC is used for
  * debug purposes by outputting information with low overhead. It is particularly useful to
  * output fast changing analog-like data as voltage references or measured currents to compare
  * them with measurements made with other means, for instance.
  *
  * The RC Timer DAC component aims at using a PWM Timer coupled with an RC network as a DAC...
  *
  * @todo Confirm and Document... Is it really relevant to deliver this?
  *
  * @{
  */

//{{{
/**
  * @brief  Hardware and software initialization of the DAC object. This is the
  *         implementation of the virtual function.
  * @param  pHandle related component instance. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  */
void DACT_Init(UI_Handle_t *pHandle)
{

  /* Output Compare PWM Mode configuration: Channel 3 & 4 */
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH4);

  /* Disable TIM4 peripheral Preload */
  LL_TIM_OC_DisablePreload(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_DisablePreload(TIM3, LL_TIM_CHANNEL_CH4);

  /* Enable TIM4 counter */
  LL_TIM_EnableCounter(TIM3);
}
//}}}
//{{{
/**
  * @brief  This method is used to update the DAC outputs. The selected
  *         variables will be provided in the related output channels. This is
  *         the implementation of the virtual function.
  * @param  pHandle related component intance. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  * @retval none.
  */
void DACT_Exec(UI_Handle_t *pHandle)
{
  MC_Protocol_REG_t bCh1_var,bCh2_var;

  DAC_UI_Handle_t *pDacHandle = (DAC_UI_Handle_t *)pHandle;

  bCh1_var = pDacHandle->bChannel_variable[DAC_CH0];
  bCh2_var = pDacHandle->bChannel_variable[DAC_CH1];

  LL_TIM_OC_SetCompareCH3 (TIM3, ((uint16_t)((int16_t)(((int16_t)UI_GetReg(pHandle,bCh1_var)+32768)/32))));
  LL_TIM_OC_SetCompareCH4 (TIM3, ((uint16_t)((int16_t)(((int16_t)UI_GetReg(pHandle,bCh2_var)+32768)/32))));
}
//}}}
