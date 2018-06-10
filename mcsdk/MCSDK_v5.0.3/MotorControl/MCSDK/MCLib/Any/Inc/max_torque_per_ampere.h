#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//}}}

#include "mc_type.h"

#define SEGMENT_NUM         ((uint8_t)7)          /* coeff no. -1 */
#define MTPA_ARRAY_SIZE     SEGMENT_NUM+1

typedef struct
{
  int16_t  SegDiv;               /**< Segments divisor */
  int32_t  AngCoeff[MTPA_ARRAY_SIZE];          /**< Angular coefficients table */
  int32_t  Offset[MTPA_ARRAY_SIZE];            /**< Offsets table */
} MTPA_Handle_t;

/*  Function used to set configure an instance of the CCC Component *****/
Curr_Components MTPA_CalcCurrRefFromIq(MTPA_Handle_t *pHandle, Curr_Components Iqdref);

//{{{
#ifdef __cplusplus
}
#endif /* __cpluplus */
//}}}
