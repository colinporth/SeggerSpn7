//{{{
/** @defgroup MC_IRQ_HANDLER Motor Control IRQ Handler
  * @brief Performs registration and execution of Interrupts handlers used for Motor Control
  *
  *  This component is a temporary work around allowing to use the former IRQ Handler
  * registration mechanism with both the old Classes and the new Components.
  *
  *  The former mechanism is defined in the MCIRQHandlerClass and is basically doing
  * two things.
  *
  *  The first one is to register an Object and a method of that object to be executed
  * when an given interrupt occurs. Interrupt sources are identified by a number; 4
  * such sources are defined:
  *
  * - #MC_IRQ_PWMNCURRFDBK_1
  * - #MC_IRQ_PWMNCURRFDBK_2
  * - #MC_IRQ_SPEEDNPOSFDBK_1
  * - #MC_IRQ_SPEEDNPOSFDBK_2
  *
  *  The second thing it does is to provide a function that executes the Interrupt handler
  * registered for a given interrupt source.
  *
  *  The implementation of the MCIRQHandlerClass mandated that the first field of the
  * structure of the registered Object be a pointer on the interrupt handling function to
  * execute.
  *
  *  This constraint is unacceptable for Components and this is the reason why this work-around
  * is proposed.
  *
  *  It is only a work around as the actual interrupt handling method to be used on the 5.0.0
  * onwards will rely on Cube FW mechanisms and is still TBD in details.
  *
  *  This work around is designed to work both with remnants of the former 4.3 architecture and
  * with the new components from the 5.0 architecture. It minimizes the changes to be made in the
  * legacy 4.3 code by providing two macros named after the MCIRQHandlerClass methods:
  *
  * - #Set_IRQ_Handler
  * - #Exec_IRQ_Handler
  *
  *  And it provides two functions, similar to the former MCIRQHandlerClass's methods, but designed
  * to work with the new components.
  * @{
  */
//}}}
#include "mc_irq_handler.h"
#include <stddef.h>

typedef struct {
  MCIRQ_Handler_t Handler;
  void*           Handle;
  } MCIRQ_HandlerConfigItem_t;

#define MCIRQ_MAX_HANDLERS 4

static MCIRQ_HandlerConfigItem_t MCIRQ_Table[MCIRQ_MAX_HANDLERS];

//{{{
/**
 * @brief Registers function @p Handler as the handler for the interrupt identified by @p IrqId.
 *  @p Handle is also registered and passed as first argument to the @p Handler function when it
 * is executed.
 */
void MCIRQ_SetIrqHandler (uint8_t IrqId, MCIRQ_Handler_t Handler, void* Handle) {

  if (IrqId < MCIRQ_MAX_HANDLERS) {
    MCIRQ_Table[IrqId].Handler = Handler;
    MCIRQ_Table[IrqId].Handle  = Handle;
    }
  }
//}}}
//{{{
/** @brief Executes the handler registered with identifier @p IrqId and returns its return value.
 *  @p Flag is passed as second argument to the handler function, the first being the pointer
 * that was registered with it.
 */
void* MCIRQ_ExecIrqHandler (uint8_t IrqId, uint8_t Flag ) {

  void* ret_val = NULL;
  if ( IrqId < MCIRQ_MAX_HANDLERS )
    ret_val = MCIRQ_Table[ IrqId ].Handler( MCIRQ_Table[ IrqId ].Handle, Flag );
  return ret_val;
  }
//}}}
