#include "bus_voltage_sensor.h"

//{{{
/**
  * @brief  It return latest Vbus conversion result expressed in u16Volt
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t
  * @retval uint16_t Latest Vbus conversion result in digit
  */
  uint16_t VBS_GetBusVoltage_d(BusVoltageSensor_Handle_t *pHandle)
  {
    return(pHandle->LatestConv);
  }
//}}}

//__attribute__((section ("ccmram")))
//{{{
/**
  * @brief  It return latest averaged Vbus measurement expressed in u16Volt
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t
  * @retval uint16_t Latest averaged Vbus measurement in digit
  */
  uint16_t VBS_GetAvBusVoltage_d(BusVoltageSensor_Handle_t *pHandle)
  {
    return(pHandle->AvBusVoltage_d);
  }
//}}}

//{{{
/**
  * @brief  It return latest averaged Vbus measurement expressed in Volts
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t
  * @retval uint16_t Latest averaged Vbus measurement in Volts
  */
  uint16_t VBS_GetAvBusVoltage_V(BusVoltageSensor_Handle_t *pHandle)
  {
    uint32_t temp;

    temp = (uint32_t)(pHandle->AvBusVoltage_d);
    temp *= pHandle->ConversionFactor;
    temp /= 65536u;

    return((uint16_t)temp);
  }
//}}}
//{{{
/**
  * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
  *         bus voltage and protection threshold values
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t
  * @retval uint16_t Fault code error
  */
  uint16_t VBS_CheckVbus(BusVoltageSensor_Handle_t *pHandle)
  {
    return(pHandle->FaultState);
  }

//}}}
