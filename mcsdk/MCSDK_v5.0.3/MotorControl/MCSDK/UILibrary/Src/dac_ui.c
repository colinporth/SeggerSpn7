#include "dac_common_ui.h"
#include "dac_ui.h"

#define DACOFF 32768

//{{{
/**
  * @brief  Hardware and software initialization of the DAC object. This is the
  *         implementation of the virtual function.
  * @param  pHandle pointer on related component instance. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  */
void DAC_Init(UI_Handle_t *pHandle)
{
  DAC_UI_Handle_t *pDacHandle = (DAC_UI_Handle_t *)pHandle;

  if(pDacHandle->hDAC_CH1_ENABLED == ENABLE)
  {
    /* Enable DAC Channel1 */
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);

  }
#if defined(DAC_CHANNEL2_SUPPORT)
  if(pDacHandle->hDAC_CH2_ENABLED == ENABLE)
  {
    /* Enable DAC Channel2 */
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_2);

  }
#endif
}
//}}}
//{{{
/**
  * @brief  This method is used to update the DAC outputs. The selected
  *         variables will be provided in the related output channels. This is
  *         the implementation of the virtual function.
  * @param  pHandle pointer on related component instance. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  */
void DAC_Exec(UI_Handle_t *pHandle)
{
  DAC_UI_Handle_t *pDacHandle = (DAC_UI_Handle_t *)pHandle;
  MC_Protocol_REG_t bCh_var;

  if(pDacHandle->hDAC_CH1_ENABLED == ENABLE)
  {
  bCh_var = pDacHandle->bChannel_variable[DAC_CH0];
    LL_DAC_ConvertData12LeftAligned(DAC1, LL_DAC_CHANNEL_1,
                    DACOFF + ((int16_t)UI_GetReg(pHandle,bCh_var)));
    LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_1);
  }
#if defined(DAC_CHANNEL2_SUPPORT)
  if(pDacHandle->hDAC_CH2_ENABLED == ENABLE)
  {
  bCh_var = pDacHandle->bChannel_variable[DAC_CH1];
  LL_DAC_ConvertData12LeftAligned(DAC1, LL_DAC_CHANNEL_2,
                     DACOFF + ((int16_t)UI_GetReg(pHandle,bCh_var)));
    LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_2);

  }
#endif
}
//}}}
