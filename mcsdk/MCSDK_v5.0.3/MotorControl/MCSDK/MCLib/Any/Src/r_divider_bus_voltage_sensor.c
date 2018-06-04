#include "r_divider_bus_voltage_sensor.h"

//{{{
/**
  * @brief  It initializes bus voltage conversion (ADC channel, conversion time,
  *         GPIO port and pin). It must be called only after PWMC_Init.
  * @param  pHandle related RDivider_Handle_t
  * @retval none
  */
void RVBS_Init (RDivider_Handle_t* pHandle, PWMC_Handle_t* PWMnCurrentSensor) {

  ADConv_t ADConv_struct;

  pHandle->PWMnCurrentSensor = PWMnCurrentSensor;

  /* Configure AD chaneel sampling time */
  ADConv_struct.Channel = pHandle->VbusADChannel;
  ADConv_struct.SamplTime = pHandle->VbusSamplingTime;
  PWMC_ADC_SetSamplingTime(PWMnCurrentSensor, ADConv_struct);
  RVBS_Clear(pHandle);
  }
//}}}
//{{{
/**
  * @brief  It clears bus voltage FW variable containing average bus voltage
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval none
  */
void RVBS_Clear (RDivider_Handle_t* pHandle) {

  uint16_t aux = (pHandle->OverVoltageThreshold + pHandle->UnderVoltageThreshold)/2u;

  uint16_t index;
  for (index = 0u; index < pHandle->LowPassFilterBW; index++)
    pHandle->aBuffer[index] = aux;

  pHandle->_Super.LatestConv = aux;
  pHandle->_Super.AvBusVoltage_d = aux;
  pHandle->index = 0;
  }
//}}}

//{{{
static uint16_t RVBS_ConvertVbusFiltrered (RDivider_Handle_t* pHandle) {

  uint16_t hAux;
  uint8_t vindex;
  uint16_t max,min;
  uint32_t tot = 0u;

  for (vindex = 0; vindex < pHandle->LowPassFilterBW;) {
    hAux = PWMC_ExecRegularConv(pHandle->PWMnCurrentSensor, pHandle->VbusADChannel);
    if (hAux != 0xFFFFu) {
      if (vindex == 0) {
        min = hAux;
        max = hAux;
        }
      else {
        if (hAux < min)
          min = hAux;
        if (hAux > max)
          max = hAux;
        }
      vindex++;
      tot += hAux;
      }
    }

  tot -= max;
  tot -= min;
  return (uint16_t)(tot / (pHandle->LowPassFilterBW-2u));
  }
//}}}
//{{{
/**
  * @brief  It actually performes the Vbus ADC conversion and updates average
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
uint16_t RVBS_CalcAvVbusFilt (RDivider_Handle_t* pHandle) {

  uint32_t wtemp;
  uint8_t i;

  uint16_t hAux = RVBS_ConvertVbusFiltrered(pHandle);
  if (hAux != 0xFFFF) {
    pHandle->aBuffer[pHandle->index] = hAux;
    wtemp = 0;
    for (i = 0; i < pHandle->LowPassFilterBW; i++)
      wtemp += pHandle->aBuffer[i];
    wtemp /= pHandle->LowPassFilterBW;
    pHandle->_Super.AvBusVoltage_d = (uint16_t)wtemp;
    pHandle->_Super.LatestConv = hAux;

    if (pHandle->index < pHandle->LowPassFilterBW-1)
      pHandle->index++;
    else
      pHandle->index = 0;
  }

  pHandle->_Super.FaultState = RVBS_CheckFaultState(pHandle);
  return(pHandle->_Super.FaultState);
  }
//}}}
//{{{
/**
  * @brief  It actually performes the Vbus ADC conversion and updates average
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
uint16_t RVBS_CalcAvVbus (RDivider_Handle_t* pHandle) {

  uint32_t wtemp;
  uint16_t hAux;
  uint8_t i;

  hAux = PWMC_ExecRegularConv(pHandle->PWMnCurrentSensor, pHandle->VbusADChannel);

  if (hAux != 0xFFFF) {
    pHandle->aBuffer[pHandle->index] = hAux;
    wtemp = 0;
    for (i = 0; i < pHandle->LowPassFilterBW; i++)
      wtemp += pHandle->aBuffer[i];
    wtemp /= pHandle->LowPassFilterBW;
    pHandle->_Super.AvBusVoltage_d = (uint16_t)wtemp;
    pHandle->_Super.LatestConv = hAux;

    if (pHandle->index < pHandle->LowPassFilterBW-1)
      pHandle->index++;
    else
      pHandle->index = 0;
  }

  pHandle->_Super.FaultState = RVBS_CheckFaultState(pHandle);
  return(pHandle->_Super.FaultState);
  }
//}}}
//{{{
/**
  * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
  *         bus voltage and protection threshold values
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
uint16_t RVBS_CheckFaultState (RDivider_Handle_t* pHandle) {

  uint16_t fault;
  if (pHandle->_Super.AvBusVoltage_d > pHandle->OverVoltageThreshold)
      fault = MC_OVER_VOLT;
    else if (pHandle->_Super.AvBusVoltage_d < pHandle->UnderVoltageThreshold)
      fault = MC_UNDER_VOLT;
    else
      fault = MC_NO_ERROR;

  return fault;
  }
//}}}
