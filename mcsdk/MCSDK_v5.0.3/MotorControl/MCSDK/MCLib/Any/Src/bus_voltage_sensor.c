#include "bus_voltage_sensor.h"

//{{{
uint16_t VBS_GetBusVoltage_d (BusVoltageSensor_Handle_t *pHandle)
{
  return(pHandle->LatestConv);
}
//}}}

#if defined (CCMRAM)
  #if defined (__ICCARM__)
    #pragma location = ".ccmram"
  #elif defined (__CC_ARM)
    __attribute__((section ("ccmram")))
  #endif
#endif

//{{{
uint16_t VBS_GetAvBusVoltage_d (BusVoltageSensor_Handle_t *pHandle)
{
  return(pHandle->AvBusVoltage_d);
}
//}}}
//{{{
uint16_t VBS_GetAvBusVoltage_V (BusVoltageSensor_Handle_t *pHandle)
{
  uint32_t temp;

  temp = (uint32_t)(pHandle->AvBusVoltage_d);
  temp *= pHandle->ConversionFactor;
  temp /= 65536u;

  return((uint16_t)temp);
}
//}}}
//{{{
uint16_t VBS_CheckVbus (BusVoltageSensor_Handle_t *pHandle)
{
  return(pHandle->FaultState);
}
//}}}
