#include "main.h"
#include "mc_tuning.h"
#include "mc_interface.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "UITask.h"

const char s_fwVer[32] = "Colins MC SDK\0Ver.5.0.3";
MCI_Handle_t* pMCI[MC_NUM];
MCT_Handle_t* pMCT[MC_NUM];
uint32_t wConfig[MC_NUM] = {UI_CONFIG_M1,UI_CONFIG_M2};

//{{{
void MX_MotorControl_Init() {

  // Reconfigure the SysTick interrupt to fire every 2 ms
  HAL_SYSTICK_Config (HAL_RCC_GetHCLKFreq() / 2000);

  // Initialize the Motor Control Subsystem
  MCboot (pMCI, pMCT);
  mc_lock_pins();

  // Initialize the MC User Interface
  UI_TaskInit (UI_INIT_CFG, wConfig, MC_NUM, pMCI, pMCT, s_fwVer);
  }
//}}}
//{{{
void MC_HandleStartStopButton() {

  if (MCI_GetSTMState(pMCI[0]) == IDLE)
    // MCI_ExecSpeedRamp(pMCI[0], 2000, 2000);
    MCI_StartMotor (pMCI[0]);
  else
    MCI_StopMotor (pMCI[0]);
  }
//}}}
