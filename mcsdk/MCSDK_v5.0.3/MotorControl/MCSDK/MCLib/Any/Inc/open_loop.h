#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//}}}
#include "mc_type.h"
#include "virtual_speed_sensor.h"

typedef struct {
  int16_t hDefaultVoltage; // Default Open loop phase voltage. */
  bool VFMode;             // Flag to enable Voltage versus Frequency mode (V/F mode). */
  int16_t hVFOffset;       // Minimum Voltage to apply when frequency is equal to zero. */
  int16_t hVFSlope;        // Slope of V/F curve: Voltage = (hVFSlope)*Frequency + hVFOffset. */
  int16_t hVoltage;        // Current Open loop phase voltage. */
  VirtualSpeedSensor_Handle_t * pVSS; // Allow access on mechanical speed measured. */
  } OpenLoop_Handle_t;

/**
  * @brief  Initialize OpenLoop variables.
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  * @param  pVSS: Pointer on virtual speed sensor structure.
  */
void OL_Init (OpenLoop_Handle_t *pHandle, VirtualSpeedSensor_Handle_t * pVSS);

/**
  * @brief  Set Vqd according to open loop phase voltage.
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  *  @retval Voltage components Vqd conditioned values.
  */
Volt_Components OL_VqdConditioning (OpenLoop_Handle_t *pHandle);

/**
  * @brief  Allow to set new open loop phase voltage.
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  * @param  hNewVoltage: New voltage value to apply.
  */
void OL_UpdateVoltage (OpenLoop_Handle_t *pHandle, int16_t hNewVoltage);

/**
  * @brief  Compute phase voltage to apply according to average mechanical speed (V/F Mode).
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  */
void OL_Calc (OpenLoop_Handle_t *pHandle);

/**
  * @brief  Allow activation of the Voltage versus Frequency mode (V/F mode).
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  * @param  VFEnabling: Flag to enable the V/F mode.
  */
void OL_VF (OpenLoop_Handle_t *pHandle, bool VFEnabling);

//{{{
#ifdef __cplusplus
}
#endif /* __cpluplus */
//}}}
