#include "ntc_temperature_sensor.h"
//{{{
/** @defgroup TemperatureSensor NTC Temperature Sensor
  * @brief Allows to read the temperature of the heat sink
  *
  * This component implements both a virtual and a real temperature sensor,
  * depending on the sensor availability.
  *
  * Access to the MCU peripherals needed to acquire the temperature (GPIO and ADC
  * used for regular conversion) is managed by the PWM component used in the Motor
  * Control subsystem. As a consequence, this NTC temperature sensor implementation
  * is hardware-independent.
  *
  * If a real temperature sensor is available (Sensor Type = #REAL_SENSOR),
  * this component can handle NTC sensors or, more generally, analog temperature sensors
  * which output is related to the temperature by the following formula:
  *
  * @f[
  *               V_{out} = V_0 + \frac{dV}{dT} \cdot ( T - T_0)
  * @f]
  *
  * In case a real temperature sensor is not available (Sensor Type = #VIRTUAL_SENSOR),
  * This component will always returns a constant, programmable, temperature.
  *
  * @{
  */
//}}}

uint16_t NTC_SetFaultState(NTC_Handle_t *pHandle);

//{{{
/**
  * @brief Returns fault when temperature exceeds the over voltage protection threshold
  *
  *  @p pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  *  @r Fault status : Updated internal fault status
  */
uint16_t NTC_SetFaultState(NTC_Handle_t *pHandle)
{
    uint16_t hFault;

    if(pHandle->hAvTemp_d > pHandle->hOverTempThreshold)
    {
        hFault = MC_OVER_TEMP;
    }
    else if (pHandle->hAvTemp_d < pHandle->hOverTempDeactThreshold)
    {
        hFault = MC_NO_ERROR;
    }
    else
    {
        hFault = pHandle->hFaultState;
    }
    return hFault;
}
//}}}

//{{{
/**
 * @brief Initializes temperature sensing conversions
 *
 *  @p pHandle : Pointer on Handle structure of TemperatureSensor component
 *
 *  @p pPWMnCurrentSensor : Handle on the PWMC component to be used for regular conversions
 */
void NTC_Init(NTC_Handle_t *pHandle, PWMC_Handle_t * pPWMnCurrentSensor)
{

    if (pHandle->bSensorType == REAL_SENSOR)
    {
        pHandle->pPWMnCurrentSensor = pPWMnCurrentSensor;

        NTC_Clear(pHandle);
    }
    else  /* case VIRTUAL_SENSOR */
    {
        pHandle->hFaultState = MC_NO_ERROR;
        pHandle->hAvTemp_d = pHandle->hExpectedTemp_d;
    }

}
//}}}
//{{{
/**
 * @brief Initializes internal average temperature computed value
 *
 *  @p pHandle : Pointer on Handle structure of TemperatureSensor component
 */
void NTC_Clear(NTC_Handle_t *pHandle)
{
    pHandle->hAvTemp_d = 0u;
}
//}}}

//{{{
/**
  * @brief Performs the temperature sensing average computation after an ADC conversion
  *
  *  @p pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  *  @r Fault status : Error reported in case of an over temperature detection
  */
uint16_t NTC_CalcAvTemp(NTC_Handle_t *pHandle)
{
    uint32_t wtemp;
    uint16_t hAux;

    if (pHandle->bSensorType == REAL_SENSOR)
    {
        hAux = PWMC_ExecRegularConv((PWMC_Handle_t *)pHandle->pPWMnCurrentSensor, pHandle->bTsensADChannel);

        if(hAux != 0xFFFFu)
        {
            wtemp =  (uint32_t)(pHandle->hLowPassFilterBW)-1u;
            wtemp *= (uint32_t) (pHandle->hAvTemp_d);
            wtemp += hAux;
            wtemp /= (uint32_t)(pHandle->hLowPassFilterBW);

            pHandle->hAvTemp_d = (uint16_t) wtemp;
        }

        pHandle->hFaultState = NTC_SetFaultState(pHandle);
    }
    else  /* case VIRTUAL_SENSOR */
    {
        pHandle->hFaultState = MC_NO_ERROR;
    }

    return(pHandle->hFaultState);
}
//}}}
//{{{
/**
  * @brief  Returns latest averaged temperature measured expressed in u16Celsius
  *
  * @p pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  * @r AverageTemperature : Current averaged temperature measured (in u16Celsius)
  */
uint16_t NTC_GetAvTemp_d(NTC_Handle_t *pHandle)
{
    return(pHandle->hAvTemp_d);
}
//}}}
//{{{
/**
  * @brief  Returns latest averaged temperature expressed in Celsius degrees
  *
  * @p pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  * @r AverageTemperature : Latest averaged temperature measured (in Celsius degrees)
  */
int16_t NTC_GetAvTemp_C(NTC_Handle_t *pHandle)
{
    int32_t wTemp;

    if (pHandle->bSensorType == REAL_SENSOR)
    {
        wTemp = (int32_t)(pHandle->hAvTemp_d);
        wTemp -= (int32_t)(pHandle->wV0);
        wTemp *= pHandle->hSensitivity;
        wTemp = wTemp/65536 + (int32_t)(pHandle->hT0);
    }
    else
    {
        wTemp = pHandle->hExpectedTemp_C;
    }
    return((int16_t)wTemp);
}
//}}}
//{{{
/**
  * @brief  Returns Temperature measurement fault status
  *
  * Fault status can be either MC_OVER_TEMP when measure exceeds the protection threshold values or
  * MC_NO_ERROR if it is inside authorized range.
  *
  * @p pHandle: Pointer on Handle structure of TemperatureSensor component.
  *
  *  @r Fault status : read internal fault state
  */
uint16_t NTC_CheckTemp(NTC_Handle_t *pHandle)
{
    return(pHandle->hFaultState);
}
//}}}
