#include "Timebase.h"

#if defined(PFC_ENABLED)
  #include "pid_regulator.h"
#endif

#include "mc_tuning.h"
#include "mc_interface.h"

#if defined(PFC_ENABLED)
  #include "PFCApplication.h"
#endif

#include "mc_tasks.h"
#include "UITask.h"
#include "parameters_conversion.h"

extern uint8_t bMCBootCompleted;

static volatile uint16_t  bUDTBTaskCounter;
static volatile uint16_t  hKey_debounce_500us = 0;

//{{{
/**
  * @brief  Use this function to know whether the user time base is elapsed
  * has elapsed
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
bool TB_UserTimebaseHasElapsed(void)
{
  bool retVal = false;
  if (bUDTBTaskCounter == 0u)
  {
    retVal = true;
  }
  return (retVal);
}
//}}}
//{{{
/**
  * @brief  It set a counter intended to be used for counting the user time base
  *         time
  * @param  SysTickCount number of System ticks to be counted
  * @retval void
  */
void TB_SetUserTimebaseTime(uint16_t SysTickCount)
{
  bUDTBTaskCounter = SysTickCount;
}
//}}}
//{{{
/**
  * @brief  It is the task scheduler.
  * @param  none
  * @retval none
  */
void TB_Scheduler(void)
{
  if ( 1 == bMCBootCompleted ) {
    MC_Scheduler(); // Moved here to let SafetyTask to overcome MF action */

    TSK_SafetyTask();

#ifdef PFC_ENABLED
    {
      PFC_Scheduler();
    }
#endif

    UI_Scheduler();

    if(bUDTBTaskCounter > 0u)
    {
      bUDTBTaskCounter--;
    }

    if (hKey_debounce_500us != 0)
    {
      hKey_debounce_500us --;
    }
  }
}
//}}}
//{{{
void TB_Set_DebounceDelay_500us(uint8_t hDelay)
{
  hKey_debounce_500us = hDelay;
}
//}}}
//{{{
bool TB_DebounceDelay_IsElapsed(void)
{
 if (hKey_debounce_500us == 0)
 {
   return (true);
 }
 else
 {
   return (false);
 }
}
//}}}
