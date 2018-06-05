#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//}}}
#include "speed_pos_fdbk.h"

typedef struct STO_Handle STO_Handle_t;

typedef void (*STO_ForceConvergency1_Cb_t)( STO_Handle_t *pHandle);
typedef void (*STO_ForceConvergency2_Cb_t)( STO_Handle_t *pHandle);
typedef void (*STO_OtfResetPLL_Cb_t)( STO_Handle_t *pHandle);
typedef bool (*STO_SpeedReliabilityCheck_Cb_t)( STO_Handle_t *pHandle);

struct STO_Handle {
  SpeednPosFdbk_Handle_t*          _Super;
  STO_ForceConvergency1_Cb_t       pFctForceConvergency1;
  STO_ForceConvergency2_Cb_t       pFctForceConvergency2;
  STO_OtfResetPLL_Cb_t             pFctStoOtfResetPLL;
  STO_SpeedReliabilityCheck_Cb_t   pFctSTO_SpeedReliabilityCheck;
  };

//{{{
#ifdef __cplusplus
}
#endif /* __cpluplus */
//}}}
