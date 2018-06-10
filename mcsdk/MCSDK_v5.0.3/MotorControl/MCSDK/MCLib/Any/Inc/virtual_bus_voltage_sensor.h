#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//}}}

#include "bus_voltage_sensor.h"

typedef struct {
  BusVoltageSensor_Handle_t _Super;
  uint16_t ExpectedVbus_d;            /*!< Expected Vbus voltage expressed in digital value
                                           hOverVoltageThreshold(digital value)= Over Voltage Threshold (V) * 65536
                                           / 500 */
  }VirtualBusVoltageSensor_Handle_t;

void VVBS_Init(VirtualBusVoltageSensor_Handle_t *pHandle);
void VVBS_Clear(VirtualBusVoltageSensor_Handle_t *pHandle);
uint16_t VVBS_NoErrors(VirtualBusVoltageSensor_Handle_t *pHandle);

//{{{
#ifdef __cplusplus
}
#endif /* __cpluplus */
//}}}
