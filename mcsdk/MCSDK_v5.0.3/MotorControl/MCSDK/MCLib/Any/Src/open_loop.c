#include "open_loop.h"

//{{{
/**
  * @brief  Initialize OpenLoop variables.
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  * @param  pVSS: Pointer on virtual speed sensor structure.
  *  @retval none
  */
void OL_Init (OpenLoop_Handle_t* pHandle, VirtualSpeedSensor_Handle_t* pVSS) {

  pHandle->hVoltage = pHandle->hDefaultVoltage;
  pHandle->pVSS = pVSS;
  }
//}}}

//{{{
/**
  * @brief  Set Vqd according to open loop phase voltage.
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  *  @retval Voltage components Vqd conditioned values.
  */
Volt_Components OL_VqdConditioning (OpenLoop_Handle_t* pHandle) {

  Volt_Components Vqd;
  Vqd.qV_Component1 = pHandle->hVoltage;
  Vqd.qV_Component2 = 0;

  return(Vqd);
  }
//}}}
//{{{
/**
  * @brief  Allow to set new open loop phase voltage.
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  * @param  hNewVoltage: New voltage value to apply.
  * @retval None
  */
void OL_UpdateVoltage (OpenLoop_Handle_t* pHandle, int16_t hNewVoltage) {
  pHandle->hVoltage = hNewVoltage;
  }
//}}}
//{{{
/**
  * @brief  Compute phase voltage to apply according to average mechanical speed (V/F Mode).
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  * @retval None
  */
void OL_Calc (OpenLoop_Handle_t* pHandle) {

  if (pHandle->VFMode == true) {
    /* V/F mode true means enabled */
    int16_t hMecSpeed01Hz = pHandle->pVSS->_Super.hAvrMecSpeed01Hz;
    int16_t hOLVoltage = pHandle->hVFOffset + (pHandle->hVFSlope*hMecSpeed01Hz);
    pHandle->hVoltage = hOLVoltage;
    }
  }
//}}}
//{{{
/**
  * @brief  Allow activation of the Voltage versus Frequency mode (V/F mode).
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  * @param  VFEnabling: Flag to enable the V/F mode.
  * @retval None
  */
void OL_VF (OpenLoop_Handle_t* pHandle, bool VFEnabling) {
  pHandle->VFMode = VFEnabling;
  }
//}}}
