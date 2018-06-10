#pragma once

//{{{
#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//}}}

#include "mc_type.h"
#include "bus_voltage_sensor.h"
#include "digital_output.h"

/*
  * @brief ICL_State_t defines all the existing ICL states of the state machine
  */
typedef enum
{
  ICL_IDLE,             /* stable state */
  ICL_ACTIVATION,       /* transition state */
  ICL_ACTIVE,           /* stable state */
  ICL_DEACTIVATION,     /* transition state */
  ICL_INACTIVE          /* stable state */
} ICL_State_t;


/**
  * @brief  ICL_Handle_t is used to handle an instance of the InrushCurrentLimiter component
  */
typedef struct
{
  BusVoltageSensor_Handle_t *pVBS;  /*!< CVBS object used for the automatic ICL component activation/deactivation */
  DOUT_handle_t *pDOUT;                  /*!< DOUT object used to physically activate/deactivate the ICL component */

  ICL_State_t ICLstate;         /*!< Current state of the ICL state machine */
  uint16_t hICLTicksCounter;    /*!< Number of clock events remaining to complete the ICL activation/deactivation */
  uint16_t hICLTotalTicks;      /*!< Total number of clock events to complete the ICL activation/deactivation */
  uint16_t hICLFrequencyHz;     /*!< Clock frequency used (Hz) to trigger the ICL_Exec() method */
  uint16_t hICLDurationms;      /*!< ICL activation/deactivation duration (ms)*/
} ICL_Handle_t;


void ICL_Init(ICL_Handle_t *pHandle, BusVoltageSensor_Handle_t *pVBS, DOUT_handle_t *pDOUT);
ICL_State_t ICL_Exec(ICL_Handle_t *pHandle);
ICL_State_t ICL_GetState(ICL_Handle_t *pHandle);

//{{{
#ifdef __cplusplus
}
#endif /* __cpluplus */
//}}}
