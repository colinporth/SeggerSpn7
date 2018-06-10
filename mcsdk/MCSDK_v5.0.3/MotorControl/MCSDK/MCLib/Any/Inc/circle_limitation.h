#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//}}}

#include "mc_type.h"

typedef struct {
  uint16_t MaxModule;               /**<  Circle limitation maximum allowed module */
  uint16_t Circle_limit_table[87];  /**<  Circle limitation table */
  uint8_t  Start_index;             /**<  Circle limitation table indexing start */
  } CircleLimitation_Handle_t;

Volt_Components Circle_Limitation(CircleLimitation_Handle_t *pHandle, Volt_Components Vqd);
//{{{
#ifdef __cplusplus
}
#endif /* __cpluplus */
//}}}
