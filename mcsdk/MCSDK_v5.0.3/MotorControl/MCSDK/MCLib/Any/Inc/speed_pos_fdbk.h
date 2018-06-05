#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//}}}
#include "stdint.h"
#include "mc_type.h"

typedef struct {
  int16_t hElAngle;
  int16_t hMecAngle;
  int16_t hAvrMecSpeed01Hz;
  int16_t hElSpeedDpp;
  int16_t hMecAccel01HzP;
  uint8_t bSpeedErrorNumber;

  uint8_t bElToMecRatio;  /*!< Coefficient used to transform electrical to
                               mechanical quantities and viceversa. It usually
                               coincides with motor pole pairs number*/
  uint16_t hMaxReliableMecSpeed01Hz; /*!< Maximum value of measured speed that is
                                        considered to be valid. It's expressed
                                        in tenth of mechanical Hertz.*/
  uint16_t hMinReliableMecSpeed01Hz; /*!< Minimum value of measured speed that is
                                        considered to be valid. It's expressed
                                        in tenth of mechanical Hertz.*/
  uint8_t bMaximumSpeedErrorsNumber; /*!< Maximum value of not valid measurements
                                        before an error is reported.*/
  uint16_t hMaxReliableMecAccel01HzP; /*!< Maximum value of measured acceleration
                                        that is considered to be valid. It's
                                        expressed in 01HzP (tenth of Hertz per
                                        speed calculation period)*/
  uint16_t hMeasurementFrequency;  /*!< Frequency on which the user will request
                                    a measurement of the rotor electrical angle.
                                    It's also used to convert measured speed from
                                    tenth of Hz to dpp and viceversa.*/
  } SpeednPosFdbk_Handle_t;

typedef struct {
  Volt_Components  Valfa_beta;
  Curr_Components  Ialfa_beta;
  uint16_t         Vbus;
  } Observer_Inputs_t;


int16_t SPD_GetElAngle(SpeednPosFdbk_Handle_t *pHandle);
int16_t SPD_GetMecAngle(SpeednPosFdbk_Handle_t *pHandle);
int16_t SPD_GetAvrgMecSpeed01Hz(SpeednPosFdbk_Handle_t *pHandle);
int16_t SPD_GetElSpeedDpp(SpeednPosFdbk_Handle_t *pHandle);

bool SPD_Check(SpeednPosFdbk_Handle_t *pHandle);

bool SPD_IsMecSpeedReliable(SpeednPosFdbk_Handle_t *pHandle, int16_t *pMecSpeed01Hz);

int16_t SPD_GetS16Speed(SpeednPosFdbk_Handle_t *pHandle);
uint8_t SPD_GetElToMecRatio(SpeednPosFdbk_Handle_t *pHandle);

void SPD_SetElToMecRatio(SpeednPosFdbk_Handle_t *pHandle, uint8_t bPP);

//{{{
#ifdef __cplusplus
}
#endif /* __cpluplus */
//}}}
