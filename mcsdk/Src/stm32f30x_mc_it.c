#include "MCIRQHandlerClass.h"
#include "UIIRQHandlerClass.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "mc_type.h"
#include "mc_tasks.h"
#include "UITask.h"
#include "mc_config.h"
#include "Timebase.h"
#include "parameters_conversion.h"
#include "r3_4_f30x_pwm_curr_fdbk.h"
#include "Timebase.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"
#include "mc_config.h"


#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
//{{{
void ADC1_2_IRQHandler()
{
  // Clear Flags Single or M1
  LL_ADC_ClearFlag_JEOS( ADC1 );
  // Highfrequency task Single or M1
  UI_DACUpdate(TSK_HighFrequencyTask());
}
//}}}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
//{{{
/**
  * @brief  This function handles ADC3 interrupt request.
  * @param  None
  * @retval None
  */
void ADC3_IRQHandler()
{
}
//}}}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
//{{{
/**
  * @brief  This function handles ADC4 interrupt request.
  * @param  None
  * @retval None
  */
void ADC4_IRQHandler()
{
}
//}}}

//{{{
/**
  * @brief  This function handles first motor TIMx Update interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_UP_M1_IRQHandler()
{
    LL_TIM_ClearFlag_UPDATE(PWM_TIMER_SELECTION);
    R3_4_F30X_TIMx_UP_IRQHandler(pwmcHandle[M1]);
}
//}}}
//{{{
void TIMx_BRK_M1_IRQHandler() {

  if (LL_TIM_IsActiveFlag_BRK (PWM_TIMER_SELECTION)) {
    LL_TIM_ClearFlag_BRK (PWM_TIMER_SELECTION);
    R3_4_F30X_BRK_IRQHandler (pwmcHandle[M1]);
    }

  if (LL_TIM_IsActiveFlag_BRK2 (PWM_TIMER_SELECTION)) {
    LL_TIM_ClearFlag_BRK2 (PWM_TIMER_SELECTION);
    R3_4_F30X_BRK2_IRQHandler (pwmcHandle[M1]);
    }

  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
  MC_Scheduler();
  }
//}}}

//{{{
/*Start here***********************************************************/
/*GUI, this section is present only if serial communication is enabled*/
/**
  * @brief  This function handles USART interrupt request.
  * @param  None
  * @retval None
  */
void USART_IRQHandler() {

  if (LL_USART_IsActiveFlag_RXNE(USART)) {
    /* Valid data have been received */
    uint16_t retVal;
    retVal = *(uint16_t*)UFCP_RX_IRQ_Handler(&pUSART,LL_USART_ReceiveData8(USART));
    if (retVal == 1)
      UI_SerialCommunicationTimeOutStart();
    if (retVal == 2)
      UI_SerialCommunicationTimeOutStop();
    }

  else if (LL_USART_IsActiveFlag_TXE(USART))
    UFCP_TX_IRQ_Handler(&pUSART);
  else if (LL_USART_IsActiveFlag_ORE(USART)) {
    /* Overrun error occurs Send Overrun message */
    UFCP_OVR_IRQ_Handler(&pUSART);
    LL_USART_ClearFlag_ORE(USART); /* Clear overrun flag */
    UI_SerialCommunicationTimeOutStop();
    }
  }
//}}}

//{{{
void HardFault_Handler() {

  TSK_HardwareFaultTask();

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1) {
    if (LL_USART_IsActiveFlag_ORE(USART)) {
      /* Overrun error occurs Send Overrun message */
      UFCP_OVR_IRQ_Handler(&pUSART);
      LL_USART_ClearFlag_ORE(USART); /* Clear overrun flag */
      UI_SerialCommunicationTimeOutStop();
      }

    else if (LL_USART_IsActiveFlag_TXE(USART))
      UFCP_TX_IRQ_Handler (&pUSART);

    else if (LL_USART_IsActiveFlag_RXNE(USART)) {
      /* Valid data have been received */
      uint16_t retVal;
      retVal = *(uint16_t*)(UFCP_RX_IRQ_Handler(&pUSART,LL_USART_ReceiveData8(USART)));
      if (retVal == 1)
        UI_SerialCommunicationTimeOutStart();
      if (retVal == 2)
        UI_SerialCommunicationTimeOutStop();
      }
    }
  }
//}}}
//{{{
void SysTick_Handler() {

  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  TB_Scheduler();
  }
//}}}
