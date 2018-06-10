#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//}}}

#include "mc_type.h"

#ifdef STM32F0XX
#include "fast_div.h"
#endif

typedef struct
{
  uint16_t FrequencyHz;             /*!< Execution frequency expressed in Hz */
  int16_t  TargetFinal;             /*!< Backup of hTargetFinal to be applied in the
                                         last step.*/
  int32_t  Ext;                     /*!< Current state variable multiplied by 32768.*/
  uint32_t RampRemainingStep;       /*!< Number of steps remaining to complete the
                                         ramp.*/
  int32_t  IncDecAmount;            /*!< Increment/decrement amount to be applied to
                                         the reference value at each
                                         CalcTorqueReference.*/
  uint32_t ScalingFactor;           /*!< Scaling factor between output value and
                                         its internal rapresentation.*/
#ifdef STM32F0XX
  /* (Fast division optimization for cortex-M0 micros)*/
  FastDiv_Handle_t fd;                       /*!< Fast division obj.*/
#endif
}RampExtMngr_Handle_t;

void REMNG_Init(RampExtMngr_Handle_t *pHandle);
int32_t REMNG_Calc(RampExtMngr_Handle_t *pHandle);
bool REMNG_ExecRamp(RampExtMngr_Handle_t *pHandle, int32_t TargetFinal, uint32_t Durationms);
int32_t REMNG_GetValue(RampExtMngr_Handle_t *pHandle);
bool REMNG_RampCompleted(RampExtMngr_Handle_t *pHandle);
void REMNG_StopRamp(RampExtMngr_Handle_t *pHandle);

//{{{
#ifdef __cplusplus
}
#endif /* __cpluplus */
//}}}
