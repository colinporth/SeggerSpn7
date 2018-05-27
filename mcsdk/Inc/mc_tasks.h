#pragma once
#include "mc_tuning.h"
#include "mc_interface.h"
#include "pwm_curr_fdbk.h"

extern PWMC_Handle_t* pwmcHandle[];

void MCboot( MCI_Handle_t* pMCIList[], MCT_Handle_t* pMCTList[] );

void MC_Scheduler(void);

void TSK_SafetyTask(void);
uint8_t TSK_HighFrequencyTask(void);
void TSK_DualDriveFIFOUpdate(void *oDrive);
void TSK_HardwareFaultTask(void);

void mc_lock_pins (void);
