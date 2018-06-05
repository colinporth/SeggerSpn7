#include "UITask.h"
#include "mc_config.h"
#include "mc_library_isr_priority_conf.h"
#include "usart_params.h"
#include "ui_exported_functions.h"
#include "parameters_conversion.h"

#define OPT_DACX  0x20 /*!<Bit field indicating that the UI uses SPI AD7303 DAC.*/

DAC_UI_Handle_t * pDAC = MC_NULL;
extern DAC_UI_Handle_t DAC_UI_Params;

MCP_Handle_t* pMCP = MC_NULL;
MCP_Handle_t MCP_UI_Params;

static volatile uint16_t  bUITaskCounter;
static volatile uint16_t  bCOMTimeoutCounter;
static volatile uint16_t  bCOMATRTimeCounter = SERIALCOM_ATR_TIME_TICKS;

#define VECT_TABLE_BASE 0x08030000

/* Setup the exported functions see UIExportedFunctions.h enum. */
void* const exportedFunctions[EF_UI_NUMBERS] = {
  (void*)(&UI_GetReg),
  (void*)(&UI_ExecSpeedRamp),
  (void*)(&UI_SetReg),
  (void*)(&UI_ExecCmd),
  (void*)(&UI_GetSelectedMCConfig),
  (void*)(&UI_SetRevupData),
  (void*)(&UI_GetRevupData),
  (void*)(&UI_SetDAC),
  (void*)(&UI_SetCurrentReferences)
  };

//{{{
void UI_TaskInit (uint8_t cfg, uint32_t* pUICfg, uint8_t bMCNum, MCI_Handle_t* pMCIList[],
                  MCT_Handle_t* pMCTList[],const char* s_fwVer ) {

  pDAC = &DAC_UI_Params;
  pDAC->_Super = UI_Params;

  UI_Init(&pDAC->_Super, bMCNum, pMCIList, pMCTList, pUICfg); /* Init UI and link MC obj */
  UI_DACInit(&pDAC->_Super); /* Init DAC */
  UI_SetDAC(&pDAC->_Super, DAC_CH0, DEFAULT_DAC_CHANNEL_1);
  UI_SetDAC(&pDAC->_Super, DAC_CH1, DEFAULT_DAC_CHANNEL_2);

  if (cfg & OPT_COM) {
    pMCP = &MCP_UI_Params;
    pMCP->_Super = UI_Params;

    UFCP_Init( & pUSART );
    MCP_Init(pMCP, (FCP_Handle_t *) & pUSART, & UFCP_Send, & UFCP_Receive, & UFCP_AbortReceive, pDAC, s_fwVer);
    UI_Init(&pMCP->_Super, bMCNum, pMCIList, pMCTList, pUICfg); /* Initialize UI and link MC components */
    }
  }
//}}}
//{{{
void UI_Scheduler() {

  if (bUITaskCounter > 0u)
    bUITaskCounter--;

  if (bCOMTimeoutCounter > 1u)
    bCOMTimeoutCounter--;

  if (bCOMATRTimeCounter > 1u)
    bCOMATRTimeCounter--;
  }
//}}}

//{{{
void UI_DACUpdate (uint8_t bMotorNbr) {

  if (UI_GetSelectedMC(&pDAC->_Super) == bMotorNbr)
    UI_DACExec(&pDAC->_Super); /* Exec DAC update */
  }
//}}}

//{{{
void MC_SetDAC (DAC_Channel_t bChannel, MC_Protocol_REG_t bVariable)
{
  UI_SetDAC(&pDAC->_Super, bChannel, bVariable);
}
//}}}
//{{{
void MC_SetUserDAC (DAC_UserChannel_t bUserChNumber, int16_t hValue)
{
  UI_SetUserDAC(&pDAC->_Super, bUserChNumber, hValue);
}
//}}}

MCP_Handle_t* GetMCP() { return pMCP; }
UI_Handle_t* GetDAC() { return &pDAC->_Super; }

//{{{
bool UI_IdleTimeHasElapsed()
{
  bool retVal = false;
  if (bUITaskCounter == 0u)
    retVal = true;
  return (retVal);
  }
//}}}
void UI_SetIdleTime(uint16_t SysTickCount) { bUITaskCounter = SysTickCount; }

//{{{
bool UI_SerialCommunicationTimeOutHasElapsed() {
  bool retVal = false;
  if (bCOMTimeoutCounter == 1u) {
    bCOMTimeoutCounter = 0u;
    retVal = true;
    }
  return (retVal);
  }
//}}}
//{{{
bool UI_SerialCommunicationATRTimeHasElapsed() {
  bool retVal = false;
  if (bCOMATRTimeCounter == 1u) {
    bCOMATRTimeCounter = 0u;
    retVal = true;
    }
  return (retVal);
  }
//}}}

//{{{
void UI_SerialCommunicationTimeOutStop() {
  bCOMTimeoutCounter = 0u;
  }
//}}}
//{{{
void UI_SerialCommunicationTimeOutStart() {
  bCOMTimeoutCounter = SERIALCOM_TIMEOUT_OCCURENCE_TICKS;
  }
//}}}
