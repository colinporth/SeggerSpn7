#include "circle_limitation.h"

#include "mc_type.h"

/**
  * @brief Check whether Vqd.qV_Component1^2 + Vqd.qV_Component2^2 <= 32767^2
  *        and if not it applies a limitation keeping constant ratio
  *        Vqd.qV_Component1 / Vqd.qV_Component2
  * @param  pHandle pointer on the related component instance
  * @param  Vqd Voltage in qd reference frame
  * @retval Volt_Components Limited Vqd vector
  */
Volt_Components Circle_Limitation(CircleLimitation_Handle_t *pHandle, Volt_Components Vqd)
{
  uint16_t table_element;
  uint32_t uw_temp;
  int32_t  sw_temp;
  Volt_Components local_vqd = Vqd;

  sw_temp = (int32_t)(Vqd.qV_Component1) * Vqd.qV_Component1 +
            (int32_t)(Vqd.qV_Component2) * Vqd.qV_Component2;

  uw_temp = (uint32_t) sw_temp;

  /* uw_temp min value 0, max value 2*32767*32767 */
  if (uw_temp > (uint32_t)(pHandle->MaxModule) * pHandle->MaxModule)
  {

    uw_temp /= (uint32_t)(16777216);

    /* wtemp min value pHandle->Start_index, max value 127 */
    uw_temp -= pHandle->Start_index;

    /* uw_temp min value 0, max value 127 - pHandle->Start_index */
    table_element = pHandle->Circle_limit_table[(uint8_t)uw_temp];

    sw_temp = Vqd.qV_Component1 * (int32_t)table_element;
    local_vqd.qV_Component1 = (int16_t)(sw_temp/32768);

    sw_temp = Vqd.qV_Component2 * (int32_t)(table_element);
    local_vqd.qV_Component2 = (int16_t)(sw_temp/32768);
  }

  return(local_vqd);
}
