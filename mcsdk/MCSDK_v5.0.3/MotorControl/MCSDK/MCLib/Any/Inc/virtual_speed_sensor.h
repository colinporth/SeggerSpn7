#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//}}}
#include "speed_pos_fdbk.h"

typedef struct {
  SpeednPosFdbk_Handle_t   _Super;
  int32_t wElAccDppP32;   /*!< Delta electrical speed expressed in dpp per speed
                               sampling period to be appied each time is called
                               SPD_CalcAvrgMecSpeed01Hz multiplied by scaling
                               factor of 65536.*/
  int32_t wElSpeedDpp32;  /*!< Electrical speed expressed in dpp multiplied by
                               scaling factor 65536.*/
  uint16_t hRemainingStep;/*!< Number of steps remaining to reach the final
                               speed.*/
  int16_t hFinalMecSpeed01Hz;/*!< Backup of hFinalMecSpeed01Hz to be applied in
                               the last step.*/
  bool bTransitionStarted;    /*!< Retaining information about Transition status.*/
  bool bTransitionEnded;      /*!< Retaining information about ransition status.*/
  int16_t hTransitionRemainingSteps;  /*!< Number of steps remaining to end
                               transition from CVSS_SPD to other CSPD*/
  int16_t hElAngleAccu;        /*!< Electrical angle accumulator*/
  bool bTransitionLocked;      /*!< Transition acceleration started*/
  bool bCopyObserver;          /*!< Command to set VSPD output same as state observer*/

  uint16_t hSpeedSamplingFreqHz; /*!< Frequency (Hz) at which motor speed is to
                             be computed. It must be equal to the frequency
                             at which function SPD_CalcAvrgMecSpeed01Hz
                             is called.*/
  int16_t hTransitionSteps; /*< Number of steps to perform the transition phase
                             from CVSS_SPD to other CSPD; if the Transition PHase
                             should last TPH milliseconds, and the FOC Execution
                             Frequency is set to FEF kHz, then
                             hTransitionSteps = TPH * FEF*/
  } VirtualSpeedSensor_Handle_t;

/* It initializes the Virtual Speed Sensor component */
void VSS_Init(VirtualSpeedSensor_Handle_t *pHandle);

/* It clears Virtual Speed Sensor by re-initializing private variables*/
void VSS_Clear(VirtualSpeedSensor_Handle_t *pHandle);

/* It compute a theorical speed and update the theorical electrical angle. */
int16_t VSS_CalcElAngle(VirtualSpeedSensor_Handle_t *pHandle, void *pInputVars_str);

/*It computes through pMecSpeed01Hz the rotor theorical average mechanical speed in 01Hz*/
bool VSS_CalcAvrgMecSpeed01Hz(VirtualSpeedSensor_Handle_t *pHandle, int16_t *hMecSpeed01Hz);

/* It set istantaneous information on VSS mechanical and  electrical angle.*/
void VSS_SetMecAngle(VirtualSpeedSensor_Handle_t *pHandle, int16_t hMecAngle);

/* Set the mechanical acceleration of virtual sensor. */
void  VSS_SetMecAcceleration(VirtualSpeedSensor_Handle_t *pHandle, int16_t  hFinalMecSpeed01Hz,
                              uint16_t hDurationms);
/* Checks if the ramp executed after a VSPD_SetMecAcceleration command has been completed*/
bool VSS_RampCompleted(VirtualSpeedSensor_Handle_t *pHandle);

/* Get the final speed of last setled ramp of virtual sensor expressed in 0.1Hz*/
int16_t  VSS_GetLastRampFinalSpeed(VirtualSpeedSensor_Handle_t *pHandle);

/* Set the command to Start the transition phase from VirtualSpeedSensor to other SpeedSensor.*/
bool VSS_SetStartTransition(VirtualSpeedSensor_Handle_t *pHandle, bool bCommand);

/* Return the status of the transition phase.*/
bool VSS_IsTransitionOngoing(VirtualSpeedSensor_Handle_t *pHandle);

/* It set istantaneous information on rotor electrical angle copied by state observer */
void VSS_SetCopyObserver(VirtualSpeedSensor_Handle_t *pHandle);

/* It  set istantaneous information on rotor electrical angle */
void VSS_SetElAngle(VirtualSpeedSensor_Handle_t *pHandle, int16_t hElAngle);
//{{{
#ifdef __cplusplus
}
#endif /* __cpluplus */
//}}}
