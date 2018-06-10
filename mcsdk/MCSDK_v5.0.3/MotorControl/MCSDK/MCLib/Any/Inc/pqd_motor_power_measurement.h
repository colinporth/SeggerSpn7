#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//}}}

#include "motor_power_measurement.h"
#include "bus_voltage_sensor.h"

typedef struct
{
	MotorPowMeas_Handle_t _super;

	int32_t wConvFact; /* It is the conversion factor used to convert the
												 variables expressed in digit into variables expressed
												 in physical measurement unit. It is used to convert the
												 power in watts. It must be equal to
												 (1000 * 3 * Vdd�)/(sqrt(3) * Rshunt * Aop) */

	pFOCVars_t pFOCVars;    /*!< Pointer to FOC vars used by MPM.*/
	BusVoltageSensor_Handle_t * pVBS;              /*!< Bus voltage sensor object used by MPM.*/
}PQD_MotorPowMeas_Handle_t;



/**
	* @brief Implementation of derived class init method. It should be called before each motor restart.
	* @param pHandle related component instance.
	* @retval none.
	*/
void PQD_Clear(PQD_MotorPowMeas_Handle_t *pHandle);

/**
	* @brief Implementation of derived class CalcElMotorPower.
	* @param pHandle related component instance.
	* @retval int16_t The measured motor power expressed in watt.
	*/
void PQD_CalcElMotorPower(PQD_MotorPowMeas_Handle_t *pHandle);

//{{{
#ifdef __cplusplus
}
#endif /* __cpluplus */
//}}}
