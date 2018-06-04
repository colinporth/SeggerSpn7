#include "motor_power_measurement.h"
#include "mc_type.h"

//{{{
/**
  * @brief  It should be called before each motor restart. It clears the
  *         measurement buffer and initialize the index.
  * @param power handle.
  * @retval none.
  */
void MPM_Clear (MotorPowMeas_Handle_t* pHandle) {
  uint16_t i;
  for (i = 0u; i < MPM_BUFFER_LENGHT; i++)
    pHandle->hMeasBuffer[i] = 0;
  pHandle->hNextMeasBufferIndex = 0u;
  pHandle->hLastMeasBufferIndex = 0u;
  }
//}}}

//{{{
/**
  * @brief  This method should be called with periodicity. It computes and
  *         returns the measured motor power expressed in watt. It is also used
  *         to fill, with that measure, the buffer used to compute the average
  *         motor power.
  * @param pHandle pointer on the related component instance.
  * @retval int16_t The measured motor power expressed in watt.
  */
int16_t MPM_CalcElMotorPower (MotorPowMeas_Handle_t* pHandle,int16_t CurrentMotorPower) {

  uint16_t i;
  int32_t wAux = 0;

  /* Store the measured values in the buffer.*/
  pHandle->hMeasBuffer[pHandle->hNextMeasBufferIndex] = CurrentMotorPower;
  pHandle->hLastMeasBufferIndex = pHandle->hNextMeasBufferIndex;
  pHandle->hNextMeasBufferIndex++;
  if (pHandle->hNextMeasBufferIndex >= MPM_BUFFER_LENGHT)
    pHandle->hNextMeasBufferIndex = 0u;

  /* Compute the average measured motor power */
  for (i = 0u; i < MPM_BUFFER_LENGHT; i++)
    wAux += (int32_t)(pHandle->hMeasBuffer[i]);
  wAux /= (int32_t)MPM_BUFFER_LENGHT;
  pHandle->hAvrgElMotorPowerW = (int16_t)(wAux);

  /* Return the last measured motor power */
  return CurrentMotorPower;
  }
//}}}
//{{{
/**
  * @brief  This method is used to get the last measured motor power
  *         (instantaneous value) expressed in watt.
  * @param pHandle pointer on the related component instance.
  * @retval int16_t The last measured motor power (instantaneous value)
  *         expressed in watt.
  */
int16_t MPM_GetElMotorPowerW (MotorPowMeas_Handle_t* pHandle) {
  return (pHandle->hMeasBuffer[pHandle->hLastMeasBufferIndex]);
  }
//}}}
//{{{
/**
  * @brief  This method is used to get the average measured motor power expressed in watt.
  * @param pHandle pointer on the related component instance.
  * @retval int16_t The average measured motor power expressed in watt.
  */
int16_t MPM_GetAvrgElMotorPowerW (MotorPowMeas_Handle_t* pHandle) {
  return (pHandle->hAvrgElMotorPowerW);
  }
//}}}
