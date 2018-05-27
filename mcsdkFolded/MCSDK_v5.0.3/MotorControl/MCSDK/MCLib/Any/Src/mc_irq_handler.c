#include "mc_irq_handler.h"
#include <stddef.h>

typedef struct {
  MCIRQ_Handler_t Handler;
  void*           Handle;
  } MCIRQ_HandlerConfigItem_t;

#define MCIRQ_MAX_HANDLERS 4

static MCIRQ_HandlerConfigItem_t MCIRQ_Table[MCIRQ_MAX_HANDLERS];

void MCIRQ_SetIrqHandler( uint8_t IrqId, MCIRQ_Handler_t Handler, void * Handle )
{
    if ( IrqId < MCIRQ_MAX_HANDLERS )
    {
        MCIRQ_Table[ IrqId ].Handler = Handler;
        MCIRQ_Table[ IrqId ].Handle  = Handle;
    }
}

void * MCIRQ_ExecIrqHandler( uint8_t IrqId, uint8_t Flag )
{
    void * ret_val = NULL;

    if ( IrqId < MCIRQ_MAX_HANDLERS )
    {
        ret_val = MCIRQ_Table[ IrqId ].Handler( MCIRQ_Table[ IrqId ].Handle, Flag );
    }

    return ret_val;
}
