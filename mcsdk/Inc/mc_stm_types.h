#ifndef __MC_STM_TYPES_H
#define __MC_STM_TYPES_H

#ifndef USE_FULL_LL_DRIVER
#define USE_FULL_LL_DRIVER
#endif

#ifdef MISRA_C_2004_BUILD
#error "The code is not ready for that..."
#endif

  #ifdef MISRA_C_2004_BUILD
    #include "stm32f30x_MisraCompliance.h"
    #define FULL_MISRA_C_COMPLIANCY
  #else
    #include "stm32f3xx_hal.h"
    #include "stm32f3xx_ll_bus.h"
    #include "stm32f3xx_ll_rcc.h"
    #include "stm32f3xx_ll_system.h"
    #include "stm32f3xx_ll_adc.h"
    #include "stm32f3xx_ll_tim.h"
    #include "stm32f3xx_ll_gpio.h"
    #include "stm32f3xx_ll_usart.h"
    #include "stm32f3xx_ll_dac.h"
    #include "stm32f3xx_ll_dma.h"
    #include "stm32f3xx_ll_comp.h"
    #include "stm32f3xx_ll_opamp.h"
  #endif

#endif /* __MC_STM_TYPES_H */
