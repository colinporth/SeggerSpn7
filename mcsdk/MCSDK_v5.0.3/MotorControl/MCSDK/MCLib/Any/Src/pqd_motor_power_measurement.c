#include "pqd_motor_power_measurement.h"
#include "mc_type.h"

/* @brief  This method should be called with periodicity. It computes and
  *         returns the measured motor power expressed in watt. It is also used
  *         to fill, with that measure, the buffer used to compute the average
  *         motor power.
  * @param power handle.
  * @retval int16_t The measured motor power expressed in watt.
  */
void PQD_CalcElMotorPower(PQD_MotorPowMeas_Handle_t *pHandle) {

  int32_t wAux,wAux2,wAux3;
  Curr_Components Iqd = pHandle->pFOCVars->Iqd;
  Volt_Components Vqd = pHandle->pFOCVars->Vqd;
  wAux = ((int32_t)Iqd.qI_Component1 * (int32_t)Vqd.qV_Component1) +
  ((int32_t)Iqd.qI_Component2 * (int32_t)Vqd.qV_Component2);
  wAux /= 65536;

  wAux2 = pHandle->wConvFact * (int32_t)VBS_GetAvBusVoltage_V(pHandle->pVBS);
  wAux2 /= 600; /* 600 is max bus voltage expressed in volt.*/

  wAux3 = wAux * wAux2;
  wAux3 *= 6; /* 6 is max bus voltage expressed in thousend of volt.*/
  wAux3 /= 10;
  wAux3 /= 65536;

  MPM_CalcElMotorPower(&pHandle->_super,wAux3);
  }
