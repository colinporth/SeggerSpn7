/*****************************************************************************
 * Copyright (c) 2014 Rowley Associates Limited.                             *
 *                                                                           *
 * This file may be distributed under the terms of the License Agreement     *
 * provided with this software.                                              *
 *                                                                           *
 * THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND, INCLUDING THE   *
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. *
 *****************************************************************************/

.macro ISR_HANDLER name=
  .section .vectors, "ax"
  .word \name
  .section .init, "ax"
  .thumb_func
  .weak \name
\name:
1: b 1b /* endless loop */
.endm

.macro ISR_RESERVED
  .section .vectors, "ax"
  .word 0
.endm

  .syntax unified
  .global reset_handler

  .section .vectors, "ax"
  .code 16
  .global _vectors

.macro DEFAULT_ISR_HANDLER name=
  .thumb_func
  .weak \name
\name:
1: b 1b /* endless loop */
.endm

_vectors:
  .word __stack_end__
  .word reset_handler

ISR_HANDLER NMI_Handler
ISR_HANDLER HardFault_Handler
ISR_RESERVED // Populate if using MemManage (MPU)
ISR_RESERVED // Populate if using Bus fault
ISR_RESERVED // Populate if using Usage fault
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_HANDLER SVC_Handler
ISR_RESERVED // Populate if using a debug monitor
ISR_RESERVED
ISR_HANDLER PendSV_Handler
ISR_HANDLER SysTick_Handler

// External interrupts start her
ISR_HANDLER WWDG_IRQHandler
ISR_HANDLER PVD_IRQHandler
ISR_HANDLER TAMP_STAMP_IRQHandler
ISR_HANDLER RTC_WKUP_IRQHandler
ISR_HANDLER FLASH_IRQHandler
ISR_HANDLER RCC_IRQHandler
ISR_HANDLER EXTI0_IRQHandler
ISR_HANDLER EXTI1_IRQHandler
ISR_HANDLER EXTI2_TSC_IRQHandler
ISR_HANDLER EXTI3_IRQHandler
ISR_HANDLER EXTI4_IRQHandler
ISR_HANDLER DMA1_Channel1_IRQHandler
ISR_HANDLER DMA1_Channel2_IRQHandler
ISR_HANDLER DMA1_Channel3_IRQHandler
ISR_HANDLER DMA1_Channel4_IRQHandler
ISR_HANDLER DMA1_Channel5_IRQHandler
ISR_HANDLER DMA1_Channel6_IRQHandler
ISR_HANDLER DMA1_Channel7_IRQHandler
ISR_HANDLER ADC1_2_IRQHandler
ISR_HANDLER USB_HP_CAN_TX_IRQHandler
ISR_HANDLER USB_LP_CAN_RX0_IRQHandler
ISR_HANDLER CAN_RX1_IRQHandler
ISR_HANDLER CAN_SCE_IRQHandler
ISR_HANDLER EXTI9_5_IRQHandler
ISR_HANDLER TIM1_BRK_TIM15_IRQHandler
ISR_HANDLER TIM1_UP_TIM16_IRQHandler
ISR_HANDLER TIM1_TRG_COM_TIM17_IRQHandler
ISR_HANDLER TIM1_CC_IRQHandler
ISR_HANDLER TIM2_IRQHandler
ISR_HANDLER TIM3_IRQHandler
ISR_HANDLER TIM4_IRQHandler
ISR_HANDLER I2C1_EV_IRQHandler
ISR_HANDLER I2C1_ER_IRQHandler
ISR_HANDLER I2C2_EV_IRQHandler
ISR_HANDLER I2C2_ER_IRQHandler
ISR_HANDLER SPI1_IRQHandler
ISR_HANDLER SPI2_IRQHandler
ISR_HANDLER USART1_IRQHandler
ISR_HANDLER USART2_IRQHandler
ISR_HANDLER USART3_IRQHandler
ISR_HANDLER EXTI15_10_IRQHandler
ISR_HANDLER RTC_Alarm_IRQHandler
ISR_HANDLER USBWakeUp_IRQHandler
ISR_HANDLER TIM8_BRK_IRQHandler
ISR_HANDLER TIM8_UP_IRQHandler
ISR_HANDLER TIM8_TRG_COM_IRQHandler
ISR_HANDLER TIM8_CC_IRQHandler
ISR_HANDLER ADC3_IRQHandler
ISR_HANDLER FMC_IRQHandler
ISR_RESERVED
ISR_RESERVED
ISR_HANDLER SPI3_IRQHandler
ISR_HANDLER UART4_IRQHandler
ISR_HANDLER UART5_IRQHandler
ISR_HANDLER TIM6_DAC_IRQHandler
ISR_HANDLER TIM7_IRQHandler
ISR_HANDLER DMA2_Channel1_IRQHandler
ISR_HANDLER DMA2_Channel2_IRQHandler
ISR_HANDLER DMA2_Channel3_IRQHandler
ISR_HANDLER DMA2_Channel4_IRQHandler
ISR_HANDLER DMA2_Channel5_IRQHandler
ISR_HANDLER ADC4_IRQHandler
ISR_RESERVED
ISR_RESERVED
ISR_HANDLER COMP1_2_3_IRQHandler
ISR_HANDLER COMP4_5_6_IRQHandler
ISR_HANDLER COMP7_IRQHandler
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_HANDLER I2C3_EV_IRQHandler
ISR_HANDLER I2C3_ER_IRQHandler
ISR_HANDLER USB_HP_IRQHandler
ISR_HANDLER USB_LP_IRQHandler
ISR_HANDLER USBWakeUp_RMP_IRQHandler
ISR_HANDLER TIM20_BRK_IRQHandler
ISR_HANDLER TIM20_UP_IRQHandler
ISR_HANDLER TIM20_TRG_COM_IRQHandler
ISR_HANDLER TIM20_CC_IRQHandler
ISR_HANDLER FPU_IRQHandler
ISR_RESERVED
ISR_RESERVED
ISR_HANDLER SPI4_IRQHandler

  .section .vectors, "ax"
_vectors_end:

  .section .init, "ax"
  .thumb_func

  reset_handler:

#ifndef __NO_SYSTEM_INIT
  ldr r0, =__SRAM_segment_end__
  mov sp, r0
  bl SystemInit
#endif

#if !defined(__SOFTFP__)
  // Enable CP11 and CP10 with CPACR |= (0xf<<20)
  movw r0, 0xED88
  movt r0, 0xE000
  ldr r1, [r0]
  orrs r1, r1, #(0xf << 20)
  str r1, [r0]
#endif

  b _start

#ifndef __NO_SYSTEM_INIT
  .thumb_func
  .weak SystemInit
SystemInit:
  bx lr
#endif
