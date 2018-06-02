#ifndef __UITASK_H
#define __UITASK_H

#include "user_interface.h"
#include "dac_rctimer_ui.h"
#include "dac_ui.h"
#include "lcd_manager_ui.h"
#include "lcd_vintage_ui.h"
#include "motor_control_protocol.h"
#include "frame_communication_protocol.h"
#include "usart_frame_communication_protocol.h"
#if 0 /* Cubify it first */
  #include "unidirectional_fast_com.h"
#endif /* 0 */
#include "ui_irq_handler.h"

/* Exported functions --------------------------------------------------------*/
void UI_TaskInit (uint8_t cfg, uint32_t* pUICfg, uint8_t bMCNum, MCI_Handle_t * pMCIList[],
                  MCT_Handle_t* pMCTList[],const char* s_fwVer);
void UI_Scheduler(void);
void UI_LCDRefresh(void);
void UI_DACUpdate(uint8_t bMotorNbr);
UI_Handle_t * GetLCD(void);
UI_Handle_t * GetDAC(void);
MCP_Handle_t * GetMCP(void);

bool UI_IdleTimeHasElapsed(void);
void UI_SetIdleTime(uint16_t SysTickCount);
bool UI_SerialCommunicationTimeOutHasElapsed(void);
bool UI_SerialCommunicationATRTimeHasElapsed(void);
void UI_SerialCommunicationTimeOutStop(void);
void UI_SerialCommunicationTimeOutStart(void);

#define LCD_LIGHT 0x01
#define LCD_FULL  0x02

#define COM_BIDIRECTIONAL  0x01
#define COM_UNIDIRECTIONAL 0x02

#endif /* __UITASK_H */
