#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//}}}

#define STM32F3
#define STM32F303xE
#define USE_HAL_DRIVER

#define __STM32F3_CMSIS_VERSION_MAIN   (0x02) /*!< [31:24] main version */
#define __STM32F3_CMSIS_VERSION_SUB1   (0x03) /*!< [23:16] sub1 version */
#define __STM32F3_CMSIS_VERSION_SUB2   (0x01) /*!< [15:8]  sub2 version */
#define __STM32F3_CMSIS_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32F3_CMSIS_VERSION        ((__STM32F3_CMSIS_VERSION_MAIN     << 24)\
                                       |(__STM32F3_CMSIS_VERSION_SUB1 << 16)\
                                       |(__STM32F3_CMSIS_VERSION_SUB2 << 8 )\
                                       |(__STM32F3_CMSIS_VERSION_RC))

#include "stm32f303xe.h"

typedef enum { RESET = 0, SET = !RESET } FlagStatus, ITStatus;

typedef enum { DISABLE = 0, ENABLE = !DISABLE } FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum { ERROR = 0, SUCCESS = !ERROR } ErrorStatus;

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

#include "stm32f3xx_hal.h"

//{{{
#ifdef __cplusplus
}
#endif /* __cplusplus */
//}}}
