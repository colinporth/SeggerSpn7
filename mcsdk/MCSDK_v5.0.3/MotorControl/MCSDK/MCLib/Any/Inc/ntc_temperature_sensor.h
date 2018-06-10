#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//}}}

#include "mc_type.h"
#include "pwm_curr_fdbk.h"

/**
  * @brief NTC_Handle_t structure used for temperature monitoring
  *
  */
typedef struct
{
  SensorType_t  bSensorType;   /**< Type of instanced temperature.
                                    This parameter can be REAL_SENSOR or VIRTUAL_SENSOR */

  uint16_t hAvTemp_d;          /**< It contains latest available average Vbus.
                                    This parameter is expressed in u16Celsius */

  uint16_t hExpectedTemp_d;    /**< Default set when no sensor available (ie virtual sensor) */

  uint16_t hExpectedTemp_C;    /**< Default value when no sensor available (ie virtual sensor).
                                    This parameter is expressed in Celsius */

  uint16_t hFaultState;        /**< Contains latest Fault code.
                                    This parameter is set to MC_OVER_TEMP or MC_NO_ERROR */

  uint8_t bTsensADChannel;     /**< ADC channel used for conversion of temperature sensor output.
                                    This parameter must be equal to ADC_CHANNEL_xx x= 0, ...,15 */

  GPIO_TypeDef* hTsensPort;    /**< GPIO port used by bVbusADChannel.
                                    This parameter must be equal to GPIOx x= A, B, ... */

  uint16_t hTsensPin;          /**< GPIO pin used by bVbusChannel.
                                    This parameter must be equal to GPIO_Pin_x x= 0, 1, ...*/

  uint8_t bTsensSamplingTime;  /**< Sampling time used for bVbusChannel AD conversion.
                                    This parameter must be equal to ADC_SampleTime_xCycles5 x= 1, 7, ...*/

  uint16_t hLowPassFilterBW;   /**< used to configure the first order software filter bandwidth.
                                    hLowPassFilterBW = NTC_CalcBusReading
                                    call rate [Hz]/ FilterBandwidth[Hz] */
  uint16_t hOverTempThreshold; /**< Represents the over voltage protection intervention threshold.
                                    This parameter is expressed in u16Celsius through formula:
                                    hOverTempThreshold =
                                    (V0[V]+dV/dT[V/°C]*(OverTempThreshold[°C] - T0[°C]))* 65536 / MCU supply voltage */
  uint16_t hOverTempDeactThreshold; /**< Temperature threshold below which an active over temperature fault is cleared.
                                         This parameter is expressed in u16Celsius through formula:
                                         hOverTempDeactThreshold =
                                         (V0[V]+dV/dT[V/°C]*(OverTempDeactThresh[°C] - T0[°C]))* 65536 / MCU supply voltage*/
  int16_t hSensitivity;        /**< NTC sensitivity
                                    This parameter is equal to MCU supply voltage [V] / dV/dT [V/°C] */
  uint32_t wV0;                /**< V0 voltage constant value used to convert the temperature into Volts.
                                    This parameter is equal V0*65536/MCU supply
                                    Used in through formula: V[V]=V0+dV/dT[V/°C]*(T-T0)[°C] */
  uint16_t hT0;                /**< T0 temperature constant value used to convert the temperature into Volts
                                    Used in through formula: V[V]=V0+dV/dT[V/°C]*(T-T0)[°C] */

  PWMC_Handle_t * pPWMnCurrentSensor;    /**< CPWMC object to be used for regular conversions */

} NTC_Handle_t;

/* Initialize temperature sensing parameters */
void NTC_Init(NTC_Handle_t *pHandle, PWMC_Handle_t *pPWMnCurrentSensor);

/* Clear static average temperature value */
void NTC_Clear(NTC_Handle_t *pHandle);

/* Temperature sensing computation */
uint16_t NTC_CalcAvTemp(NTC_Handle_t *pHandle);

/* Get averaged temperature measurement expressed in u16Celsius */
uint16_t NTC_GetAvTemp_d(NTC_Handle_t *pHandle);

/* Get averaged temperature measurement expressed in Celsius degrees */
int16_t NTC_GetAvTemp_C(NTC_Handle_t *pHandle);

/* Get the temperature measurement fault status */
uint16_t NTC_CheckTemp(NTC_Handle_t *pHandle);

//{{{
#ifdef __cplusplus
}
#endif /* __cpluplus */
//}}}
