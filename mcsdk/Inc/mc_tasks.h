#pragma once

#include "mc_tuning.h"
#include "mc_interface.h"
#include "pwm_curr_fdbk.h"

extern PWMC_Handle_t* pwmcHandle[];

void MC_Scheduler();
void MCboot (MCI_Handle_t* pMCIList[], MCT_Handle_t* pMCTList[] );

uint8_t TSK_HighFrequencyTask();
void TSK_SafetyTask();
void TSK_HardwareFaultTask();

void mc_lock_pins (void);
