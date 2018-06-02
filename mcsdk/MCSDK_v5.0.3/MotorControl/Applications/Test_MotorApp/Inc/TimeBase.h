#ifndef __TIMEBASE_H
#define __TIMEBASE_H

#include "mc_type.h"

bool TB_UserTimebaseHasElapsed();
void TB_SetUserTimebaseTime (uint16_t SysTickCount);
void TB_Scheduler();

void TB_Set_DebounceDelay_500us (uint8_t hDelay);
bool TB_DebounceDelay_IsElapsed();

#endif /* __MCBOOTSINGLEMOTOR_H */
