// 3 shunt resistors current sensing topology, 2 ADC peripherals - 2 motors 2 instances, 4 ADCs
#include "r3_4_f30x_pwm_curr_fdbk.h"
#include "mc_type.h"

#define TIMxCCER_MASK_CH123     ((uint32_t)  0x00000555u)
#define NB_CONVERSIONS          16u
#define CCMR2_CH4_DISABLE       0x8FFFu
#define CCMR2_CH4_PWM1          0x6000u
#define CCMR2_CH4_PWM2          0x7000u
#define OPAMP_CSR_DEFAULT_MASK  ((uint32_t)0xFFFFFF93u)

//{{{
// Stores into the component's handle the voltage present on Ia and
// Ib current feedback analog channels when no current is flowing into the motor
// pHandle handler of the current instance of the PWM component
__attribute__((section ("ccmram"))) static uint16_t R3_4_F30X_WriteTIMRegisters (PWMC_Handle_t* pHdl) {

  uint16_t hAux, hCCR4Aux;
  PWMC_R3_4_F3_Handle_t* pHandle = (PWMC_R3_4_F3_Handle_t*)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  pR3_4_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;

  LL_TIM_OC_SetCompareCH1 (TIMx,pHandle->_Super.hCntPhA);
  LL_TIM_OC_SetCompareCH2 (TIMx,pHandle->_Super.hCntPhB);
  LL_TIM_OC_SetCompareCH3 (TIMx,pHandle->_Super.hCntPhC);
  hCCR4Aux = (uint16_t)LL_TIM_OC_GetCompareCH4(TIMx);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_SetCompareCH4 (TIMx,0xFFFFu);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_SetCompareCH4 (TIMx, hCCR4Aux);

  /* Switch Context */
  if (pDOPAMPParams_str) {
    pDOPAMPParams_str->wOPAMP_Selection->CSR = pHandle->wOAMP1CR;
    pDOPAMPParams_str->wOPAMP2_Selection->CSR = pHandle->wOAMP2CR;
    }
  pHandle->pParams_str->ADCx_1->JSQR = pHandle->wADC1_JSQR;
  pHandle->pParams_str->ADCx_2->JSQR = pHandle->wADC2_JSQR;

  /* Limit for update event */
  /* Check the status flag. If an update event has occurred before to set new
  values of regs the FOC rate is too high */
  if (LL_TIM_IsActiveFlag_UPDATE(TIMx))
    hAux = MC_FOC_DURATION;
  else
    hAux = MC_NO_ERROR;

  if (pHandle->_Super.SWerror == 1u) {
    hAux = MC_FOC_DURATION;
    pHandle->_Super.SWerror = 0u;
    }

  return hAux;
  }
//}}}
//{{{
// Configure the ADC for the current sampling during calibration.
// It means set the sampling point via TIMx_Ch4 value and polarity
// ADC sequence length and channels.
// And call the WriteTIMRegisters method.
// pHandle: handler of the current instance of the PWM component
__attribute__((section ("ccmram"))) static uint16_t R3_4_F30X_SetADCSampPointCalibration (PWMC_Handle_t* pHdl) {

  PWMC_R3_4_F3_Handle_t* pHandle = (PWMC_R3_4_F3_Handle_t*)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

  TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
  return R3_4_F30X_WriteTIMRegisters (&pHandle->_Super);
  }
//}}}
//{{{
// It computes and return latest converted motor phase currents motor during RL detection phase
// pHandle Pointer on the target component instance
// Ia and Ib current in Curr_Components format
__attribute__((section ("ccmram"))) static void R3_4_F30X_RLGetPhaseCurrents (PWMC_Handle_t* pHdl, Curr_Components* pStator_Currents) {

  PWMC_R3_4_F3_Handle_t *pHandle = (PWMC_R3_4_F3_Handle_t *)pHdl;
  ADC_TypeDef* ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef* ADCx_2 = pHandle->pParams_str->ADCx_2;
  int16_t hCurrA = 0, hCurrB = 0;

  /* Clear the flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE (pHandle->pParams_str->TIMx);

  int32_t wAux = (int32_t)(pHandle->wPhaseBOffset);
  wAux -= (int32_t)(ADCx_1->JDR1);

  /* Check saturation */
  if (wAux > -INT16_MAX) {
    if (wAux < INT16_MAX) {
      }
    else
      wAux = INT16_MAX;
    }
  else 
    wAux = -INT16_MAX;

  hCurrA = (int16_t)(wAux);
  wAux = (int32_t)(pHandle->wPhaseBOffset);
  wAux -= (int32_t)(ADCx_2->JDR1);

  /* Check saturation */
  if (wAux > -INT16_MAX) {
    if (wAux < INT16_MAX) {
      }
    else
      wAux = INT16_MAX;
    }
  else
    wAux = -INT16_MAX;

  hCurrB = (int16_t)(wAux);

  pStator_Currents->qI_Component1 = hCurrA;
  pStator_Currents->qI_Component2 = hCurrB;

  pHandle->_Super.hIa = hCurrA;
  pHandle->_Super.hIb = hCurrB;
  pHandle->_Super.hIc = -hCurrA - hCurrB;
  }
//}}}

//{{{
/**
 * @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during
 *         calibration. It sum up injected conversion data into wPhaseAOffset and
 *         wPhaseBOffset to compute the offset introduced in the current feedback
 *         network. It is requied to proper configure ADC inputs before to enable the offset computation.
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval It always returns {0,0} in Curr_Components format
 */
static void R3_4_F30X_HFCurrentsCalibrationAB (PWMC_Handle_t* pHdl, Curr_Components* pStator_Currents) {

  PWMC_R3_4_F3_Handle_t* pHandle = (PWMC_R3_4_F3_Handle_t*)pHdl;

  /* Clear the flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->TIMx);

  if (pHandle->bIndex < NB_CONVERSIONS) {
    pHandle-> wPhaseAOffset += pHandle->pParams_str->ADCx_1->JDR1;
    pHandle-> wPhaseBOffset += pHandle->pParams_str->ADCx_2->JDR1;
    pHandle->bIndex++;
    }
  }
//}}}
//{{{
/**
 * @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during
 *         calibration. It sum up injected conversion data into wPhaseCOffset
 *         to compute the offset introduced in the current feedback
 *         network. It is requied to proper configure ADC input before to enable the offset computation.
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval It always returns {0,0} in Curr_Components format
 */
static void R3_4_F30X_HFCurrentsCalibrationC (PWMC_Handle_t* pHdl,Curr_Components* pStator_Currents) {

  PWMC_R3_4_F3_Handle_t* pHandle = (PWMC_R3_4_F3_Handle_t*)pHdl;

  /* Clear the flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->TIMx);

  if (pHandle->bIndex < NB_CONVERSIONS) {
    pHandle-> wPhaseCOffset += pHandle->pParams_str->ADCx_2->JDR1;
    pHandle->bIndex++;
    }
  }
//}}}
//{{{
/**
 * @brief  It is used to configure the analog output used for protection thresholds.
 * @param  DAC_Channel: the selected DAC channel.
 *          This parameter can be:
 *            @arg LL_DAC_CHANNEL_1: DAC Channel1 selected
 *            @arg LL_DAC_CHANNEL_2: DAC Channel2 selected
 * @param  hDACVref Value of DAC reference expressed as 16bit unsigned integer.
 *         Ex. 0 = 0V 65536 = VDD_DAC.
 */
static void R3_4_F30X_SetAOReferenceVoltage (uint32_t DAC_Channel, uint16_t hDACVref) {

  if (DAC_Channel == LL_DAC_CHANNEL_2)
    LL_DAC_ConvertData12LeftAligned (DAC1, LL_DAC_CHANNEL_2, hDACVref );
  else
    LL_DAC_ConvertData12LeftAligned (DAC1, LL_DAC_CHANNEL_1, hDACVref );

  /* Enable DAC Channel */
  LL_DAC_TrigSWConversion (DAC1, DAC_Channel);
  LL_DAC_Enable (DAC1, DAC_Channel );
  }
//}}}
//{{{
static uint32_t R3_4_F30X_ADC_InjectedChannelConfig (ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank,
                                                     uint8_t ADC_SampleTime, uint8_t SequencerLength,
                                                     uint16_t ADC_ExternalTriggerInjectedPolarity, 
                                                     uint16_t ADC_ExternalTriggerInjected) {

  uint32_t tmpreg1 = 0u, tmpreg2 = 0u, tmpregA = 0u;
  uint32_t wAux,wAux2;

  /*  ADC_InjectedSequencerLengthConfig(ADCx,1); */
  tmpregA = ADCx->JSQR;
  /* Clear the old injected sequnence lenght JL bits */
  tmpregA &= ~(uint32_t)ADC_JSQR_JL;
  /* Set the injected sequnence lenght JL bits */
  tmpregA |= ((uint32_t)(SequencerLength) - 1u); /* first value is sequencer lenght */

  /* Disable the selected ADC conversion on external event */
  tmpregA &= ~ADC_JSQR_JEXTEN;
  tmpregA |= ADC_ExternalTriggerInjectedPolarity;

  /* Disable the selected ADC conversion on external event */
  tmpregA &= ~ADC_JSQR_JEXTSEL;
  tmpregA |= ADC_ExternalTriggerInjected;

  /* Channel sampling configuration */
  /* if ADC_CHANNEL_10 ... ADC_CHANNEL_18 is selected */
  if (ADC_Channel > MC_ADC_CHANNEL_9) {
    /* Get the old register value */
    tmpreg1 = ADCx->SMPR2;
    /* Calculate the mask to clear */
    wAux = ADC_SMPR2_SMP10;
    wAux2 = 3u * ((uint32_t)(ADC_Channel) - 10u);
    tmpreg2 = wAux << wAux2;
    /* Clear the old channel sample time */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    wAux = (uint32_t)(ADC_SampleTime);
    tmpreg2 = wAux << wAux2;
    /* Set the new channel sample time */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADCx->SMPR2 = tmpreg1;
    }
  else if (ADC_Channel != 0u) {
    /* Get the old register value */
    tmpreg1 = ADCx->SMPR1;
    /* Calculate the mask to clear */
    wAux = ADC_SMPR1_SMP0;
    wAux2 = 3u * (uint32_t)(ADC_Channel);
    tmpreg2 =  wAux << wAux2;
    /* Clear the old channel sample time */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    wAux = (uint32_t)ADC_SampleTime;
    wAux2 = 3u * (uint32_t)(ADC_Channel);
    tmpreg2 =  wAux << wAux2;
    /* Set the new channel sample time */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADCx->SMPR1 = tmpreg1;
    }

  /* Rank configuration */
  /* Get the old register value */
  tmpreg1 = tmpregA;
  /* Calculate the mask to clear */
  wAux = ADC_JSQR_JSQ1;
  wAux2 = 6u * ((uint32_t)(Rank) - 1u);
  tmpreg2 = wAux << wAux2;
  /* Clear the old SQx bits for the selected rank */
  tmpreg1 &= ~tmpreg2;
  /* Calculate the mask to set */
  wAux = ADC_Channel;
  wAux2 = 6u * (uint32_t)(Rank) + 2u;
  tmpreg2 = wAux << wAux2;
  /* Set the SQx bits for the selected rank */
  tmpreg1 |= tmpreg2;
  /* Store the new register value */

  return (tmpreg1);
  }
//}}}
//{{{
/**
 * @brief  It turns on low sides switches. This function is intended to be
 *         used for charging boot capacitors of driving section. It has to be
 *         called each motor start-up when using high voltage drivers.
 *         This function is specific for RL detection phase.
 * @param  pHandle Pointer on the target component instance
 */
static void R3_4_F30X_RLTurnOnLowSides (PWMC_Handle_t* pHdl) {

  PWMC_R3_4_F3_Handle_t* pHandle = (PWMC_R3_4_F3_Handle_t*)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = true;

  /*Turn on the phase A low side switch */
  LL_TIM_OC_SetCompareCH1 (TIMx, 0u);

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Wait until next update */
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0) {}

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);

  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO) {
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }

  return;
  }
//}}}
//{{{
/**
 * @brief  It enables PWM generation on the proper Timer peripheral
 *         This function is specific for RL detection phase.
 * @param  pHandle Pointer on the target component instance
 */
static void R3_4_F30X_RLSwitchOnPWM (PWMC_Handle_t* pHdl) {

  PWMC_R3_4_F3_Handle_t* pHandle = (PWMC_R3_4_F3_Handle_t*)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  pR3_4_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0) {}

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE (TIMx);

  TIMx->CCR1 = 1u;
  TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 5u;
  pHandle->pParams_str->ADCx_2->JSQR = pHandle->wADC2_JSQR;

  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0) {}

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE ;
  LL_TIM_EnableAllOutputs (TIMx);

  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO) {
    if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u) {
      LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
      }
    else {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
      }
    }

  pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
  pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phB;

  /* Switch Context */
  if (pDOPAMPParams_str) {
    pDOPAMPParams_str->wOPAMP_Selection->CSR = pHandle->wOAMP1CR;
    pDOPAMPParams_str->wOPAMP2_Selection->CSR = pHandle->wOAMP2CR;
    }

  pHandle->pParams_str->ADCx_1->JSQR = pHandle->wADC1_JSQR;
  pHandle->pParams_str->ADCx_2->JSQR = pHandle->wADC2_JSQR;
  return;
  }
//}}}
//{{{
/**
 * @brief  It disables PWM generation on the proper Timer peripheral acting on MOE bit
 *         This function is specific for RL detection phase.
 * @param  pHandle Pointer on the target component instance
 */
static void R3_4_F30X_RLSwitchOffPWM (PWMC_Handle_t* pHdl) {

  PWMC_R3_4_F3_Handle_t* pHandle = (PWMC_R3_4_F3_Handle_t*)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef* ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef* ADCx_2 = pHandle->pParams_str->ADCx_2;

  pHandle->_Super.bTurnOnLowSidesAction = false;
  /* Main PWM Output Disable */
  if (pHandle->BrakeActionLock == true) {
    }
  else {
    TIMx->BDTR &= ~((uint32_t)(LL_TIM_OSSI_ENABLE ));
    if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO) {
      LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
      }
    }
  LL_TIM_DisableAllOutputs (TIMx);

  LL_ADC_DisableIT_JEOS(ADCx_1);

  /* Flushing JSQR queue of context by setting JADSTP = 1 (JQM)=1 */
  ADCx_1->CR |= ADC_CR_JADSTP;
  ADCx_2->CR |= ADC_CR_JADSTP;

  ADCx_1->JSQR = R3_4_F30X_ADC_InjectedChannelConfig (
    ADCx_1, 0u, 1u, 0u, 1u, LL_ADC_INJ_TRIG_EXT_RISING, pHandle->ADC_ExternalTriggerInjected);
  ADCx_2->JSQR = R3_4_F30X_ADC_InjectedChannelConfig (
    ADCx_2, 0u, 1u, 0u, 1u, LL_ADC_INJ_TRIG_EXT_RISING, pHandle->ADC_ExternalTriggerInjected);
  ADCx_1->CR |= ADC_CR_JADSTART;
  ADCx_2->CR |= ADC_CR_JADSTART;

  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;
  TIMx->CCR4 = 0xFFFFu;
  TIMx->CCR4 = 0x0u;
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);

  while (LL_ADC_IsActiveFlag_JEOS (ADCx_1) == 0) {}
  while (LL_ADC_IsActiveFlag_JEOS (ADCx_2) == 0) {}

  /* ADCx_1 Injected conversions end interrupt enabling */
  LL_ADC_ClearFlag_JEOS (ADCx_1);
  LL_ADC_ClearFlag_JEOS (ADCx_2);
  LL_ADC_EnableIT_JEOS (ADCx_1);
  return;
  }
//}}}

//{{{
/**
 * @brief  It initializes TIMx, ADC, GPIO, and NVIC for current reading in three shunt topology using STM32F30x
 * @param  pHandle: handler of the current instance of the PWM component
 */
void R3_4_F30X_Init (PWMC_R3_4_F3_Handle_t* pHandle) {

  pR3_4_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef* ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef* ADCx_2 = pHandle->pParams_str->ADCx_2;
  COMP_TypeDef* COMP_OCPAx = pHandle->pParams_str->wCompOCPASelection;
  COMP_TypeDef* COMP_OCPBx = pHandle->pParams_str->wCompOCPBSelection;
  COMP_TypeDef* COMP_OCPCx = pHandle->pParams_str->wCompOCPCSelection;
  COMP_TypeDef* COMP_OVPx = pHandle->pParams_str->wCompOVPSelection;

  if ((uint32_t)pHandle == (uint32_t)&pHandle->_Super) {
    /* disable IT and flags in case of LL driver usage
     * workaround for unwanted interrupt enabling done by LL driver */
    LL_ADC_DisableIT_EOC (ADCx_1);
    LL_ADC_ClearFlag_EOC (ADCx_1);
    LL_ADC_DisableIT_JEOC (ADCx_1);
    LL_ADC_ClearFlag_JEOC (ADCx_1);
    LL_ADC_DisableIT_EOC (ADCx_2);
    LL_ADC_ClearFlag_EOC (ADCx_2);
    LL_ADC_DisableIT_JEOC (ADCx_2);
    LL_ADC_ClearFlag_JEOC (ADCx_2);
    LL_ADC_DisableIT_EOC (pHandle->pParams_str->regconvADCx);
    LL_ADC_ClearFlag_EOC (pHandle->pParams_str->regconvADCx);
    LL_ADC_DisableIT_JEOC (pHandle->pParams_str->regconvADCx);
    LL_ADC_ClearFlag_JEOC (pHandle->pParams_str->regconvADCx);

    /* disable main TIM counter to ensure a synchronous start by TIM2 trigger */
    LL_TIM_DisableCounter (TIMx);

    if (TIMx == TIM1)
      pHandle->ADC_ExternalTriggerInjected = LL_ADC_INJ_TRIG_EXT_TIM1_TRGO;
    else
      pHandle->ADC_ExternalTriggerInjected = LL_ADC_INJ_TRIG_EXT_TIM8_TRGO;

    if ((pHandle->pParams_str->bBKIN2Mode) != NONE)
      LL_TIM_ClearFlag_BRK2 (TIMx);
    LL_TIM_EnableIT_BRK (TIMx);

    if(TIMx == TIM1)
      /* TIM1 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph (LL_DBGMCU_APB2_GRP1_TIM1_STOP);
    else
      /* TIM8 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph (LL_DBGMCU_APB2_GRP1_TIM8_STOP);

    /* Prepare timer for synchronization */
    LL_TIM_GenerateEvent_UPDATE(TIMx);
    if (pHandle->pParams_str->bFreqRatio == 2u) {
      if (pHandle->pParams_str->bIsHigherFreqTim == HIGHER_FREQ) {
        if (pHandle->pParams_str->bRepetitionCounter == 3u) {
          /* Set TIMx repetition counter to 1 */
          LL_TIM_SetRepetitionCounter (TIMx, 1);
          LL_TIM_GenerateEvent_UPDATE (TIMx);
          /* Repetition counter will be set to 3 at next Update */
          LL_TIM_SetRepetitionCounter (TIMx, 3);
          }
        }
      LL_TIM_SetCounter (TIMx, (uint32_t)(pHandle->Half_PWMPeriod)-1u);
      }
    else {
      /* bFreqRatio equal to 1 or 3 */
      if (pHandle->_Super.bMotor == M1) {
        if(pHandle->pParams_str->bRepetitionCounter == 1u)
          LL_TIM_SetCounter (TIMx, (uint32_t)(pHandle->Half_PWMPeriod)-1u);
        else if (pHandle->pParams_str->bRepetitionCounter == 3u) {
          /* Set TIMx repetition counter to 1 */
          LL_TIM_SetRepetitionCounter (TIMx, 1);
          LL_TIM_GenerateEvent_UPDATE (TIMx);
          /* Repetition counter will be set to 3 at next Update */
          LL_TIM_SetRepetitionCounter (TIMx, 3);
          }
        }
      }

    if (pDOPAMPParams_str) {
      LL_OPAMP_Enable (pDOPAMPParams_str->wOPAMP_Selection);
      LL_OPAMP_Enable (pDOPAMPParams_str->wOPAMP2_Selection);
      }

    /* Over current protection phase A */
    if (COMP_OCPAx) {
      /* Inverting input*/
      if (pHandle->pParams_str->bCompOCPAInvInput_MODE != EXT_MODE) {
        if (LL_COMP_GetInputMinus (COMP_OCPAx) == LL_COMP_INPUT_MINUS_DAC1_CH1)
          R3_4_F30X_SetAOReferenceVoltage (LL_DAC_CHANNEL_1, (uint16_t)(pHandle->pParams_str->hDAC_OCP_Threshold));
        else if (LL_COMP_GetInputMinus (COMP_OCPAx) == LL_COMP_INPUT_MINUS_DAC1_CH2)
          R3_4_F30X_SetAOReferenceVoltage (LL_DAC_CHANNEL_2, (uint16_t)(pHandle->pParams_str->hDAC_OCP_Threshold));
        }

      /* Wait to stabilize DAC voltage */
      {
        volatile uint16_t waittime = 0u;
        for (waittime = 0u; waittime < 1000u; waittime++) {}
      }

      /* Output */
      LL_COMP_Enable (COMP_OCPAx);
      LL_COMP_Lock (COMP_OCPAx);
      }

    /* Over current protection phase B */
    if (COMP_OCPBx) {
      /* Output */
      LL_COMP_Enable (COMP_OCPBx);
      LL_COMP_Lock (COMP_OCPBx);
      }

    /* Over current protection phase C */
    if (COMP_OCPCx) {
      /* Output */
      LL_COMP_Enable (COMP_OCPCx);
      LL_COMP_Lock (COMP_OCPCx);
      }

    /* Over voltage protection */
    if (COMP_OVPx) {
      /* Inverting input*/
      if (pHandle->pParams_str->bCompOVPInvInput_MODE != EXT_MODE) {
        if (LL_COMP_GetInputMinus (COMP_OVPx) == LL_COMP_INPUT_MINUS_DAC1_CH1)
          R3_4_F30X_SetAOReferenceVoltage(LL_DAC_CHANNEL_1, (uint16_t)(pHandle->pParams_str->hDAC_OVP_Threshold));
        else if (LL_COMP_GetInputMinus (COMP_OVPx) == LL_COMP_INPUT_MINUS_DAC1_CH2)
          R3_4_F30X_SetAOReferenceVoltage (LL_DAC_CHANNEL_2, (uint16_t)(pHandle->pParams_str->hDAC_OVP_Threshold));
        }

      /* Wait to stabilize DAC voltage */
      {
        volatile uint16_t waittime = 0u;
        for (waittime = 0u; waittime < 1000u; waittime++) {}
      }

      /* Output */
      LL_COMP_Enable (COMP_OVPx);
      LL_COMP_Lock (COMP_OVPx);
      }

    LL_ADC_EnableInternalRegulator (ADCx_1);
    LL_ADC_EnableInternalRegulator (ADCx_2);

    /* Wait for Regulator Startup time, once for both */
    uint16_t waittime = 0u;
    for (waittime = 0u; waittime < 65000u; waittime++) {}

    LL_ADC_StartCalibration(ADCx_1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing (ADCx_1)) {}

    LL_ADC_StartCalibration(ADCx_2,LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing (ADCx_2)) {}

    if (pHandle->_Super.bMotor == M1) {
      if ((pHandle->pParams_str->regconvADCx != ADCx_1) && (pHandle->pParams_str->regconvADCx != ADCx_2)) {
          LL_ADC_EnableInternalRegulator (pHandle->pParams_str->regconvADCx);

        /* Wait for Regulator Startup time, once for both */
        {
          uint16_t waittime = 0u;
          for(waittime=0u;waittime<65000u;waittime++) {}
        }

        LL_ADC_StartCalibration(pHandle->pParams_str->regconvADCx,LL_ADC_SINGLE_ENDED);
        while (LL_ADC_IsCalibrationOnGoing(pHandle->pParams_str->regconvADCx) ) {}
        }
      }

    /* ADCx_1 and ADCx_2 registers configuration, Enable ADCx_1 and ADCx_2 */
    LL_ADC_Enable (ADCx_1);
    LL_ADC_Enable (ADCx_2);

    if (pHandle->_Super.bMotor == M1) {
      if ((pHandle->pParams_str->regconvADCx != ADCx_1) && (pHandle->pParams_str->regconvADCx != ADCx_2))
        LL_ADC_Enable (pHandle->pParams_str->regconvADCx);
      }

    /* reset regular conversion sequencer length set by cubeMX */
    LL_ADC_REG_SetSequencerLength (pHandle->pParams_str->regconvADCx, LL_ADC_REG_SEQ_SCAN_DISABLE);

    pHandle->wADC_JSQR_phA = R3_4_F30X_ADC_InjectedChannelConfig (ADCx_1, pHandle->pParams_str->bIaChannel, 1u, pHandle->pParams_str->b_IaSamplingTime, 1u, LL_ADC_INJ_TRIG_EXT_RISING, pHandle->ADC_ExternalTriggerInjected);
    pHandle->wADC_JSQR_phB = R3_4_F30X_ADC_InjectedChannelConfig (ADCx_2, pHandle->pParams_str->bIbChannel, 1u, pHandle->pParams_str->b_IbSamplingTime, 1u, LL_ADC_INJ_TRIG_EXT_RISING, pHandle->ADC_ExternalTriggerInjected);
    pHandle->wADC_JSQR_phC = R3_4_F30X_ADC_InjectedChannelConfig (ADCx_2, pHandle->pParams_str->bIcChannel, 1u, pHandle->pParams_str->b_IcSamplingTime, 1u, LL_ADC_INJ_TRIG_EXT_RISING, pHandle->ADC_ExternalTriggerInjected);
    R3_4_F30X_ADC_InjectedChannelConfig (ADCx_2, pHandle->pParams_str->bIaChannel, 1u, pHandle->pParams_str->b_IaSamplingTime, 1u, LL_ADC_INJ_TRIG_EXT_RISING, pHandle->ADC_ExternalTriggerInjected);
    R3_4_F30X_ADC_InjectedChannelConfig (ADCx_1, pHandle->pParams_str->bIbChannel, 1u, pHandle->pParams_str->b_IbSamplingTime, 1u, LL_ADC_INJ_TRIG_EXT_RISING, pHandle->ADC_ExternalTriggerInjected);
    R3_4_F30X_ADC_InjectedChannelConfig (ADCx_1, pHandle->pParams_str->bIcChannel, 1u, pHandle->pParams_str->b_IcSamplingTime, 1u, LL_ADC_INJ_TRIG_EXT_RISING, pHandle->ADC_ExternalTriggerInjected);

    if (pHandle->pParams_str->pOPAMPParams) {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phB;
      }
    LL_ADC_INJ_SetQueueMode (ADCx_1, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY);
    LL_ADC_INJ_SetQueueMode (ADCx_2, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY);


    /* Clear the flags */
    pHandle->OverVoltageFlag = false;
    pHandle->OverCurrentFlag = false;

    pHandle->_Super.DTTest = 0u;
    pHandle->_Super.DTCompCnt = pHandle->_Super.hDTCompCnt;
    }
  }
//}}}
//{{{
/**
 * @brief  It stores into handler the voltage present on Ia and
 *         Ib current feedback analog channels when no current is flowin into the motor
 * @param  pHandle: handler of the current instance of the PWM component
 */
void R3_4_F30X_CurrentReadingCalibration (PWMC_Handle_t* pHdl)
{
  PWMC_R3_4_F3_Handle_t *pHandle = (PWMC_R3_4_F3_Handle_t *)pHdl;
  pR3_4_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  uint16_t hCalibrationPeriodCounter;
  uint16_t hMaxPeriodsNumber;

  pHandle->wPhaseAOffset = 0u;
  pHandle->wPhaseBOffset = 0u;
  pHandle->wPhaseCOffset = 0u;

  pHandle->bIndex = 0u;

  /* It forces inactive level on TIMx CHy and CHyN */
  TIMx->CCER &= (~TIMxCCER_MASK_CH123);

  /* Offset calibration for A & B phases */
  /* Change function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R3_4_F30X_HFCurrentsCalibrationAB;
  pHandle->_Super.pFctSetADCSampPointSect1 = &R3_4_F30X_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect2 = &R3_4_F30X_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect3 = &R3_4_F30X_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect4 = &R3_4_F30X_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect5 = &R3_4_F30X_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect6 = &R3_4_F30X_SetADCSampPointCalibration;

  if (pDOPAMPParams_str)
  {
    pHandle->wOAMP1CR = pDOPAMPParams_str->wOPAMP_Selection->CSR;
    pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
    pHandle->wOAMP1CR |= pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHA;

    pHandle->wOAMP2CR = pDOPAMPParams_str->wOPAMP2_Selection->CSR;
    pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
    pHandle->wOAMP2CR |= pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHB;
  }
  else
  {
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
    pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phB;
  }

  R3_4_F30X_SwitchOnPWM(&pHandle->_Super);

  /* Wait for NB_CONVERSIONS to be executed */
  hMaxPeriodsNumber=(NB_CONVERSIONS+1u)*(((uint16_t)(pHandle->pParams_str->bRepetitionCounter)+1u)>>1);
  LL_TIM_ClearFlag_CC1 (TIMx);
  hCalibrationPeriodCounter = 0u;
  while (pHandle->bIndex < NB_CONVERSIONS)
  {
    if ( LL_TIM_IsActiveFlag_CC1(TIMx))
    {
      LL_TIM_ClearFlag_CC1 (TIMx);
      hCalibrationPeriodCounter++;
      if (hCalibrationPeriodCounter >= hMaxPeriodsNumber)
      {
        if (pHandle->bIndex < NB_CONVERSIONS)
        {
          pHandle->_Super.SWerror = 1u;
          break;
        }
      }
    }
  }

  R3_4_F30X_SwitchOffPWM(&pHandle->_Super);

  /* Offset calibration for C phase */
  /* Reset bIndex */
  pHandle->bIndex=0u;

  /* Change function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R3_4_F30X_HFCurrentsCalibrationC;

  if (pDOPAMPParams_str)
  {
    pHandle->wOAMP2CR = pDOPAMPParams_str->wOPAMP2_Selection->CSR;
    pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
    pHandle->wOAMP2CR |= pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHC;
  }
  else
  {
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phC;
  }

  R3_4_F30X_SwitchOnPWM(&pHandle->_Super);

  /* Wait for NB_CONVERSIONS to be executed */
  LL_TIM_ClearFlag_CC1 (TIMx);
  hCalibrationPeriodCounter = 0u;
  while (pHandle->bIndex < NB_CONVERSIONS)
  {
    if (LL_TIM_IsActiveFlag_CC1(TIMx))
    {
      LL_TIM_ClearFlag_CC1 (TIMx);
      hCalibrationPeriodCounter++;
      if (hCalibrationPeriodCounter >= hMaxPeriodsNumber)
      {
        if (pHandle->bIndex < NB_CONVERSIONS)
        {
          pHandle->_Super.SWerror = 1u;
          break;
        }
      }
    }
  }

  R3_4_F30X_SwitchOffPWM(&pHandle->_Super);

  pHandle->wPhaseAOffset >>=4;
  pHandle->wPhaseBOffset >>=4;
  pHandle->wPhaseCOffset >>=4;

  /* Change back function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R3_4_F30X_GetPhaseCurrents;
  pHandle->_Super.pFctSetADCSampPointSect1 = &R3_4_F30X_SetADCSampPointSect1;
  pHandle->_Super.pFctSetADCSampPointSect2 = &R3_4_F30X_SetADCSampPointSect2;
  pHandle->_Super.pFctSetADCSampPointSect3 = &R3_4_F30X_SetADCSampPointSect3;
  pHandle->_Super.pFctSetADCSampPointSect4 = &R3_4_F30X_SetADCSampPointSect4;
  pHandle->_Super.pFctSetADCSampPointSect5 = &R3_4_F30X_SetADCSampPointSect5;
  pHandle->_Super.pFctSetADCSampPointSect6 = &R3_4_F30X_SetADCSampPointSect6;

  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to
     force 50% duty cycle on the three inverer legs */
  /* Disable TIMx preload */
  TIMx->CCMR1 &= 0xF7F7u;
  TIMx->CCMR2 &= 0xF7F7u;
  //LL_TIM_WriteReg(TIMx,,);
  //LL_TIM_WriteReg(TIMx,,);
  LL_TIM_OC_SetCompareCH1(TIMx,pHandle->Half_PWMPeriod);
  LL_TIM_OC_SetCompareCH2(TIMx,pHandle->Half_PWMPeriod);
  LL_TIM_OC_SetCompareCH3(TIMx,pHandle->Half_PWMPeriod);

  /* Enable TIMx preload */
  TIMx->CCMR1 |= 0x0808u;
  TIMx->CCMR2 |= 0x0808u;

  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  TIMx->CCER |= TIMxCCER_MASK_CH123;

  pHandle->BrakeActionLock = false;
}
//}}}

//{{{
/**
 * @brief  It turns on low sides switches. This function is intended to be
 *         used for charging boot capacitors of driving section. It has to be
 *         called each motor start-up when using high voltage drivers
 * @param  pHandle: handler of the current instance of the PWM component
 */
void R3_4_F30X_TurnOnLowSides (PWMC_Handle_t* pHdl) {

  PWMC_R3_4_F3_Handle_t* pHandle = (PWMC_R3_4_F3_Handle_t *)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = true;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1 (TIMx,0u);
  LL_TIM_OC_SetCompareCH2 (TIMx,0u);
  LL_TIM_OC_SetCompareCH3 (TIMx,0u);

  /* Wait until next update */
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0) {}

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);

  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO) {
    /* Enable signals activation */
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
  }
//}}}
//{{{
/**
 * @brief  It enables PWM generation on the proper Timer peripheral acting on MOE *         bit
 * @param  pHandle: handler of the current instance of the PWM component
 */
void R3_4_F30X_SwitchOnPWM (PWMC_Handle_t* pHdl) {

  PWMC_R3_4_F3_Handle_t *pHandle = (PWMC_R3_4_F3_Handle_t *)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef* ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef* ADCx_2 = pHandle->pParams_str->ADCx_2;
  pR3_4_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0) {}

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Set all duty to 50% */
  if (pHandle->_Super.RLDetectionMode == true) {
    LL_TIM_OC_SetCompareCH1 (TIMx,1u);
    ADCx_2->JSQR = pHandle->wADC2_JSQR;
    }
  else
    LL_TIM_OC_SetCompareCH1 (TIMx,(uint32_t)(pHandle->Half_PWMPeriod) >> 1);
  LL_TIM_OC_SetCompareCH2 (TIMx,(uint32_t)(pHandle->Half_PWMPeriod) >> 1);
  LL_TIM_OC_SetCompareCH3 (TIMx,(uint32_t)(pHandle->Half_PWMPeriod) >> 1);
  LL_TIM_OC_SetCompareCH4 (TIMx,(uint32_t)(pHandle->Half_PWMPeriod) - 5u);

  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0) {}

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  LL_TIM_EnableAllOutputs (TIMx);

  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO) {
    if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u) {
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
      }
    else {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
      }
    }

  /* Switch Context */
  if (pDOPAMPParams_str) {
    pDOPAMPParams_str->wOPAMP_Selection->CSR = pHandle->wOAMP1CR;
    pDOPAMPParams_str->wOPAMP2_Selection->CSR = pHandle->wOAMP2CR;
    }
  ADCx_1->JSQR = pHandle->wADC1_JSQR;
  if (pHandle->_Super.RLDetectionMode == false)
    ADCx_2->JSQR = pHandle->wADC2_JSQR;
  }
//}}}
//{{{
/**
 * @brief  It disables PWM generation on the proper Timer peripheral acting on MOE bit
 * @param  pHandle: handler of the current instance of the PWM component
 */
void R3_4_F30X_SwitchOffPWM (PWMC_Handle_t* pHdl) {

  PWMC_R3_4_F3_Handle_t *pHandle = (PWMC_R3_4_F3_Handle_t *)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  if (pHandle->BrakeActionLock == true) {
    }
  else {
    TIMx->BDTR &= ~((uint32_t)(LL_TIM_OSSI_ENABLE));

    if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO) {
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
      }
    }
  LL_TIM_DisableAllOutputs(TIMx);

  /* Flushing JSQR queue of context by setting JADSTP = 1 (JQM)=1 */
  LL_ADC_INJ_StopConversion(pHandle->pParams_str->ADCx_1);
  LL_ADC_INJ_StopConversion(pHandle->pParams_str->ADCx_2);

  LL_ADC_DisableIT_JEOS(pHandle->pParams_str->ADCx_1);

  pHandle->pParams_str->ADCx_1->JSQR = R3_4_F30X_ADC_InjectedChannelConfig(
    pHandle->pParams_str->ADCx_1, 0u, 1u, 0u, 1u, LL_ADC_INJ_TRIG_EXT_RISING,
    pHandle->ADC_ExternalTriggerInjected);
  pHandle->pParams_str->ADCx_2->JSQR = R3_4_F30X_ADC_InjectedChannelConfig(
    pHandle->pParams_str->ADCx_2, 0u, 1u, 0u, 1u, LL_ADC_INJ_TRIG_EXT_RISING,
    pHandle->ADC_ExternalTriggerInjected);

  LL_ADC_INJ_StartConversion(pHandle->pParams_str->ADCx_1);
  LL_ADC_INJ_StartConversion(pHandle->pParams_str->ADCx_2);

  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;
  TIMx->CCR4 = 0xFFFFu;
  TIMx->CCR4 = 0x0u;
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);

  while (LL_ADC_IsActiveFlag_JEOS(pHandle->pParams_str->ADCx_1) == 0) {}
  while (LL_ADC_IsActiveFlag_JEOS(pHandle->pParams_str->ADCx_2) == 0) {}

  /* ADCx_1 Injected conversions end interrupt enabling */
  LL_ADC_ClearFlag_JEOS(pHandle->pParams_str->ADCx_1);
  LL_ADC_ClearFlag_JEOS(pHandle->pParams_str->ADCx_2);
  LL_ADC_EnableIT_JEOS(pHandle->pParams_str->ADCx_1);
  }
//}}}

//{{{
/**
 * @brief  It computes and return latest converted motor phase currents motor
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval Ia and Ib current in Curr_Components format
 */
__attribute__((section ("ccmram"))) void R3_4_F30X_GetPhaseCurrents (PWMC_Handle_t* pHdl, Curr_Components* pStator_Currents)
{
  PWMC_R3_4_F3_Handle_t *pHandle = (PWMC_R3_4_F3_Handle_t*)pHdl;
  ADC_TypeDef* ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef* ADCx_2 = pHandle->pParams_str->ADCx_2;

  /* Clear the flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE (pHandle->pParams_str->TIMx);

  uint16_t hReg1 = (uint16_t)(ADCx_1->JDR1);
  uint16_t hReg2 = (uint16_t)(ADCx_2->JDR1);

  int32_t wAux;
  uint8_t bSector = (uint8_t)pHandle->_Super.hSector;
  switch (bSector) {
    case SECTOR_4:
    //{{{
    case SECTOR_5:
      /* Current on Phase C is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      wAux = (int32_t)(pHandle->wPhaseAOffset)-(int32_t)(hReg1);

      /* Saturation of Ia */
      if (wAux < -INT16_MAX)
        pStator_Currents->qI_Component1= -INT16_MAX;
      else  if (wAux > INT16_MAX)
        pStator_Currents->qI_Component1= INT16_MAX;
      else
        pStator_Currents->qI_Component1= (int16_t)wAux;

      /* Ib = PhaseBOffset - ADC converted value) */
      wAux = (int32_t)(pHandle->wPhaseBOffset)-(int32_t)(hReg2);

      /* Saturation of Ib */
      if (wAux < -INT16_MAX)
        pStator_Currents->qI_Component2= -INT16_MAX;
      else  if (wAux > INT16_MAX)
        pStator_Currents->qI_Component2= INT16_MAX;
      else
        pStator_Currents->qI_Component2= (int16_t)wAux;
      break;
    //}}}
    case SECTOR_6:
    //{{{
    case SECTOR_1:
      /* Current on Phase A is not accessible     */
      /* Ib = PhaseBOffset - ADC converted value) */
      wAux = (int32_t)(pHandle->wPhaseBOffset)-(int32_t)(hReg1);

      /* Saturation of Ib */
      if (wAux < -INT16_MAX)
        pStator_Currents->qI_Component2= -INT16_MAX;
      else  if (wAux > INT16_MAX)
        pStator_Currents->qI_Component2= INT16_MAX;
      else
        pStator_Currents->qI_Component2= (int16_t)wAux;

      /* Ia = -Ic -Ib */
      wAux = (int32_t)(hReg2) - (int32_t)(pHandle->wPhaseCOffset); /* -Ic */
      wAux -= (int32_t)pStator_Currents->qI_Component2;               /* Ia  */

      /* Saturation of Ia */
      if (wAux> INT16_MAX)
        pStator_Currents->qI_Component1 = INT16_MAX;
      else  if (wAux <-INT16_MAX)
        pStator_Currents->qI_Component1 = -INT16_MAX;
      else
        pStator_Currents->qI_Component1 = (int16_t)wAux;
      break;
    //}}}
    case SECTOR_2:
    //{{{
    case SECTOR_3:
      /* Current on Phase B is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      wAux = (int32_t)(pHandle->wPhaseAOffset)-(int32_t)(hReg1);

      /* Saturation of Ia */
      if (wAux < -INT16_MAX)
        pStator_Currents->qI_Component1= -INT16_MAX;
      else  if (wAux > INT16_MAX)
        pStator_Currents->qI_Component1= INT16_MAX;
      else
        pStator_Currents->qI_Component1= (int16_t)wAux;

      /* Ib = -Ic -Ia */
      wAux = (int32_t)(hReg2) - (int32_t)(pHandle->wPhaseCOffset); /* -Ic */
      wAux -= (int32_t)pStator_Currents->qI_Component1;               /* Ib */

      /* Saturation of Ib */
      if (wAux> INT16_MAX)
        pStator_Currents->qI_Component2=INT16_MAX;
      else  if (wAux <-INT16_MAX)
        pStator_Currents->qI_Component2 = -INT16_MAX;
      else
        pStator_Currents->qI_Component2 = (int16_t)wAux;
      break;
    //}}}
    default:
      break;
    }

  pHandle->_Super.hIa = pStator_Currents->qI_Component1;
  pHandle->_Super.hIb = pStator_Currents->qI_Component2;
  pHandle->_Super.hIc = -pStator_Currents->qI_Component1 - pStator_Currents->qI_Component2;
  }
//}}}

//{{{
/*  Configure the ADC for the current sampling related to sector 1.
 * It means set the sampling point via TIMx_Ch4 value and polarity  ADC sequence length and channels.
 * And call the WriteTIMRegisters method */
__attribute__((section ("ccmram"))) uint16_t R3_4_F30X_SetADCSampPointSect1 (PWMC_Handle_t* pHdl) {

  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_4_F3_Handle_t *pHandle = (PWMC_R3_4_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  pR3_4_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter)) {
    TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    if (pDOPAMPParams_str) {
      pHandle->wOAMP1CR = pDOPAMPParams_str->wOPAMP_Selection->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHA;

      pHandle->wOAMP2CR = pDOPAMPParams_str->wOPAMP2_Selection->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHB;
      }
    else {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phB;
      }
    }
  else {
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter)
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    else {
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhB);

      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA)*2u)
        hCntSmp = pHandle->_Super.hCntPhA - pHandle->pParams_str->hTbefore;
      else {
        hCntSmp = pHandle->_Super.hCntPhA + pHandle->pParams_str->hTafter;
        if (hCntSmp >= pHandle->Half_PWMPeriod) {
          /* Set CC4 as PWM mode 1 */
          TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
          TIMx->CCMR2 |= CCMR2_CH4_PWM1;
          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
          }
        }
      }

    if (pDOPAMPParams_str) {
      pHandle->wOAMP1CR = pDOPAMPParams_str->wOPAMP_Selection->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHB;

      pHandle->wOAMP2CR = pDOPAMPParams_str->wOPAMP2_Selection->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHC;
      }
    else {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phB;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phC;
      }

    /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp;
    }

  return R3_4_F30X_WriteTIMRegisters (&pHandle->_Super);
  }
//}}}
//{{{
/**
 * @brief  Configure the ADC for the current sampling related to sector 2.
 *         It means set the sampling point via TIMx_Ch4 value and polarity
 *         ADC sequence length and channels.
 *         And call the WriteTIMRegisters method.
 */
__attribute__((section ("ccmram"))) uint16_t R3_4_F30X_SetADCSampPointSect2 (PWMC_Handle_t* pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_4_F3_Handle_t *pHandle = (PWMC_R3_4_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  pR3_4_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
          ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = pDOPAMPParams_str->wOPAMP_Selection->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHA;

      pHandle->wOAMP2CR = pDOPAMPParams_str->wOPAMP2_Selection->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHB;
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phB;
    }
  }
  else
  {
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhA);

      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhB - pHandle->pParams_str->hTbefore;
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhB + pHandle->pParams_str->hTafter;

        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* Set CC4 as PWM mode 1 */
          TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
          TIMx->CCMR2 |= CCMR2_CH4_PWM1;

          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }

    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = pDOPAMPParams_str->wOPAMP_Selection->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHA;

      pHandle->wOAMP2CR = pDOPAMPParams_str->wOPAMP2_Selection->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHC;
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phC;
    }

    /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp;
  }
  return R3_4_F30X_WriteTIMRegisters(&pHandle->_Super);
}
//}}}
//{{{
/**
 * @brief  Configure the ADC for the current sampling related to sector 3.
 *         It means set the sampling point via TIMx_Ch4 value and polarity
 *         ADC sequence length and channels.
 *         And call the WriteTIMRegisters method.
 */
__attribute__((section ("ccmram"))) uint16_t R3_4_F30X_SetADCSampPointSect3 (PWMC_Handle_t* pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_4_F3_Handle_t *pHandle = (PWMC_R3_4_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  pR3_4_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
          ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = pDOPAMPParams_str->wOPAMP_Selection->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHA;

      pHandle->wOAMP2CR = pDOPAMPParams_str->wOPAMP2_Selection->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHB;

    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phB;
    }
  }
  else
  {
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhC);

      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhB - pHandle->pParams_str->hTbefore;
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhB + pHandle->pParams_str->hTafter;

        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* Set CC4 as PWM mode 1 */
          TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
          TIMx->CCMR2 |= CCMR2_CH4_PWM1;

          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }

    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = pDOPAMPParams_str->wOPAMP_Selection->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHA;

      pHandle->wOAMP2CR = pDOPAMPParams_str->wOPAMP2_Selection->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHC;
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phC;
    }

    /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp;
  }
  return R3_4_F30X_WriteTIMRegisters(&pHandle->_Super);
}
//}}}
//{{{
/**
 * @brief  Configure the ADC for the current sampling related to sector 4.
 *         It means set the sampling point via TIMx_Ch4 value and polarity
 *         ADC sequence length and channels.
 *         And call the WriteTIMRegisters method.
 */
__attribute__((section ("ccmram"))) uint16_t R3_4_F30X_SetADCSampPointSect4 (PWMC_Handle_t* pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_4_F3_Handle_t *pHandle = (PWMC_R3_4_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  pR3_4_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
          ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = pDOPAMPParams_str->wOPAMP_Selection->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHA;

      pHandle->wOAMP2CR = pDOPAMPParams_str->wOPAMP2_Selection->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHB;
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phB;
    }
  }
  else
  {
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhB);

      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhC - pHandle->pParams_str->hTbefore;
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhC + pHandle->pParams_str->hTafter;

        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* Set CC4 as PWM mode 1 */
          TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
          TIMx->CCMR2 |= CCMR2_CH4_PWM1;

          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }

    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = pDOPAMPParams_str->wOPAMP_Selection->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHA;

      pHandle->wOAMP2CR = pDOPAMPParams_str->wOPAMP2_Selection->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHB;
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phB;
    }

    /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp;
  }
  return R3_4_F30X_WriteTIMRegisters(&pHandle->_Super);
}
//}}}
//{{{
/**
 * @brief  Configure the ADC for the current sampling related to sector 5.
 *         It means set the sampling point via TIMx_Ch4 value and polarity
 *         ADC sequence length and channels.
 *         And call the WriteTIMRegisters method.
 */
__attribute__((section ("ccmram"))) uint16_t R3_4_F30X_SetADCSampPointSect5 (PWMC_Handle_t* pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_4_F3_Handle_t *pHandle = (PWMC_R3_4_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  pR3_4_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
          ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = pDOPAMPParams_str->wOPAMP_Selection->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHA;

      pHandle->wOAMP2CR = pDOPAMPParams_str->wOPAMP2_Selection->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHB;
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phB;
    }
  }
  else
  {
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhA);

      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhC - pHandle->pParams_str->hTbefore;
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhC + pHandle->pParams_str->hTafter;

        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* Set CC4 as PWM mode 1 */
          TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
          TIMx->CCMR2 |= CCMR2_CH4_PWM1;

          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }

    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = pDOPAMPParams_str->wOPAMP_Selection->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHA;

      pHandle->wOAMP2CR = pDOPAMPParams_str->wOPAMP2_Selection->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHB;
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phB;
    }

    /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp;
  }
  return R3_4_F30X_WriteTIMRegisters(&pHandle->_Super);
}
//}}}
//{{{
/**
 * @brief  Configure the ADC for the current sampling related to sector 6.
 *         It means set the sampling point via TIMx_Ch4 value and polarity
 *         ADC sequence length and channels.
 *         And call the WriteTIMRegisters method.
 */
__attribute__((section ("ccmram"))) uint16_t R3_4_F30X_SetADCSampPointSect6 (PWMC_Handle_t* pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_4_F3_Handle_t *pHandle = (PWMC_R3_4_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  pR3_4_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
          ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = pDOPAMPParams_str->wOPAMP_Selection->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHA;

      pHandle->wOAMP2CR = pDOPAMPParams_str->wOPAMP2_Selection->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHB;
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phB;
    }
  }
  else
  {
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhC);

      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhA - pHandle->pParams_str->hTbefore;
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhA + pHandle->pParams_str->hTafter;

        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* Set CC4 as PWM mode 1 */
          TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
          TIMx->CCMR2 |= CCMR2_CH4_PWM1;

          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }

    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = pDOPAMPParams_str->wOPAMP_Selection->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHB;

      pHandle->wOAMP2CR = pDOPAMPParams_str->wOPAMP2_Selection->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHC;
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phB;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phC;
    }

    /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp;
  }
  return R3_4_F30X_WriteTIMRegisters(&pHandle->_Super);
}
//}}}

//{{{
/**
 * @brief  It contains the TIMx Update event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 */
__attribute__((section ("ccmram"))) void* R3_4_F30X_TIMx_UP_IRQHandler (PWMC_Handle_t* pHdl) {

  PWMC_R3_4_F3_Handle_t* pHandle = (PWMC_R3_4_F3_Handle_t*)pHdl;
  return &(pHandle->_Super.bMotor);
  }
//}}}
//{{{
/**
 * @brief  It contains the TIMx Break2 event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 */
__attribute__((section ("ccmram"))) void* R3_4_F30X_BRK2_IRQHandler (PWMC_Handle_t* pHdl) {

  PWMC_R3_4_F3_Handle_t* pHandle = (PWMC_R3_4_F3_Handle_t*)pHdl;

  if (pHandle->BrakeActionLock == false) {
    if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO) {
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
      }
    }

  pHandle->OverCurrentFlag = true;
  return &(pHandle->_Super.bMotor);
  }
//}}}
//{{{
/**
 * @brief  It contains the TIMx Break1 event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
__attribute__((section ("ccmram"))) void* R3_4_F30X_BRK_IRQHandler (PWMC_Handle_t* pHdl) {

  PWMC_R3_4_F3_Handle_t* pHandle = (PWMC_R3_4_F3_Handle_t*)pHdl;

  pHandle->pParams_str->TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  pHandle->OverVoltageFlag = true;
  pHandle->BrakeActionLock = true;

  return &(pHandle->_Super.bMotor);
  }
//}}}

//{{{
/**
 * @brief  Execute a regular conversion using ADCx.
 *         The function is not re-entrant (can't executed twice at the same time)
 * @param  pHandle Pointer on the target component instance
 * @retval It returns converted value or oxFFFF for conversion error
 */
uint16_t R3_4_F30X_ExecRegularConv (PWMC_Handle_t* pHdl, uint8_t bChannel) {

  PWMC_R3_4_F3_Handle_t* pHandle = (PWMC_R3_4_F3_Handle_t *)pHdl;
  ADC_TypeDef* ADCx = pHandle->pParams_str->regconvADCx;

  LL_ADC_REG_SetSequencerRanks(ADCx, LL_ADC_REG_RANK_1, __LL_ADC_DECIMAL_NB_TO_CHANNEL(bChannel));

  LL_ADC_REG_ReadConversionData12(ADCx);
  LL_ADC_REG_StartConversion(ADCx);

  /* Wait until end of regular conversion */
  while (LL_ADC_IsActiveFlag_EOC(ADCx) == 0u) {}

  pHandle->hRegConv = LL_ADC_REG_ReadConversionData12(ADCx);
  return (pHandle->hRegConv);
  }
//}}}
//{{{
/**
 * @brief  It sets the specified sampling time for the specified ADC channel
 *         on ADCx. It must be called once for each channel utilized by user
 * @param  ADC channel, sampling time
 */
void R3_4_F30X_ADC_SetSamplingTime (PWMC_Handle_t* pHdl, ADConv_t ADConv_struct) {

  PWMC_R3_4_F3_Handle_t *pHandle = (PWMC_R3_4_F3_Handle_t *)pHdl;
  uint32_t tmpreg2 = 0u;
  uint8_t ADC_Channel = ADConv_struct.Channel;
  uint8_t ADC_SampleTime = ADConv_struct.SamplTime;

  /* Channel sampling configuration */
  if (ADC_Channel > MC_ADC_CHANNEL_9) {
    /* if ADC_CHANNEL_10 ... ADC_CHANNEL_18 is selected */
    uint32_t wAux,wAux2;
    /* Get the old register value */
    /* Calculate the mask to clear */
    wAux = ADC_SMPR2_SMP10;
    wAux2 = 3u * ((uint32_t)(ADC_Channel) - 10u);
    tmpreg2 =  wAux << wAux2;
    /* Clear the old channel sample time */
    pHandle->pParams_str->regconvADCx->SMPR2 &= ~tmpreg2;
    /* Calculate the mask to set */
    wAux = (uint32_t)(ADC_SampleTime);
    pHandle->pParams_str->regconvADCx->SMPR2 |=  wAux << wAux2;
    }
  else {
    /* ADC_Channel include in ADC_CHANNEL_[0..9] */
    uint32_t wAux,wAux2;
    /* Get the old register value */
    /* Calculate the mask to clear */
    wAux = ADC_SMPR1_SMP1;
    wAux2 = 3u * ((uint32_t)(ADC_Channel) - 1u);
    tmpreg2 =  wAux << wAux2;
    /* Clear the old channel sample time */
    pHandle->pParams_str->regconvADCx->SMPR1 &= ~tmpreg2;
    /* Calculate the mask to set */
    wAux = (uint32_t)(ADC_SampleTime);
    wAux2 = 3u * ((uint32_t)(ADC_Channel));
    pHandle->pParams_str->regconvADCx->SMPR1 |= wAux << wAux2;
    }
  }
//}}}
//{{{
/**
 * @brief  It is used to check if an overcurrent occurred since last call.
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
 *                  detected since last method call, MC_NO_FAULTS otherwise.
 */
uint16_t R3_4_F30X_IsOverCurrentOccurred (PWMC_Handle_t* pHdl) {

  PWMC_R3_4_F3_Handle_t* pHandle = (PWMC_R3_4_F3_Handle_t*)pHdl;
  uint16_t retVal = MC_NO_FAULTS;

  if (pHandle->OverVoltageFlag == true) {
    retVal = MC_OVER_VOLT;
    pHandle->OverVoltageFlag = false;
    }

  if (pHandle->OverCurrentFlag == true ) {
    retVal |= MC_BREAK_IN;
    pHandle->OverCurrentFlag = false;
    }

  return retVal;
  }
//}}}

//{{{
/**
 * @brief  It is used to set the PWM mode for R/L detection.
 * @param  pHandle: handler of the current instance of the PWM component
 * @param  hDuty to be applied in uint16_t
 */
void R3_4_F30X_RLDetectionModeEnable (PWMC_Handle_t* pHdl) {

  PWMC_R3_4_F3_Handle_t *pHandle = (PWMC_R3_4_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  if (pHandle->_Super.RLDetectionMode == false) {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode (TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
    LL_TIM_OC_SetCompareCH1(TIMx, 0u);

    /*  Channel2 configuration */
    if ((pHandle->pParams_str-> LowSideOutputs)== LS_PWM_TIMER) {
      LL_TIM_OC_SetMode (TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_ACTIVE);
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2);
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
      }
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO) {
      LL_TIM_OC_SetMode (TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_INACTIVE);
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2);
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
      }

    /*  Channel3 configuration */
    LL_TIM_OC_SetMode (TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM2);
    LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3N);

    pHandle->wADC_JSQR_phA = R3_4_F30X_ADC_InjectedChannelConfig (
      pHandle->pParams_str->ADCx_1, pHandle->pParams_str->bIbChannel, 1u,
      pHandle->pParams_str->b_IbSamplingTime, 1u, LL_ADC_INJ_TRIG_EXT_RISING,
      pHandle->ADC_ExternalTriggerInjected);
    pHandle->wADC_JSQR_phB = R3_4_F30X_ADC_InjectedChannelConfig (
      pHandle->pParams_str->ADCx_2, pHandle->pParams_str->bIbChannel, 1u,
      pHandle->pParams_str->b_IbSamplingTime, 1u, LL_ADC_INJ_TRIG_EXT_RISING,
      ADC_JSQR_JEXTSEL_3);

    pHandle->wPhaseAOffset = pHandle->wPhaseBOffset; /* Use only the offset of phB */
    }

  pHandle->_Super.pFctGetPhaseCurrents = &R3_4_F30X_RLGetPhaseCurrents;
  pHandle->_Super.pFctTurnOnLowSides = &R3_4_F30X_RLTurnOnLowSides;
  pHandle->_Super.pFctSwitchOnPwm = &R3_4_F30X_RLSwitchOnPWM;
  pHandle->_Super.pFctSwitchOffPwm = &R3_4_F30X_RLSwitchOffPWM;
  pHandle->_Super.RLDetectionMode = true;
  }
//}}}
//{{{
/**
 * @brief  It is used to disable the PWM mode for R/L detection.
 * @param  pHandle: handler of the current instance of the PWM component
 */
void R3_4_F30X_RLDetectionModeDisable (PWMC_Handle_t* pHdl) {

  PWMC_R3_4_F3_Handle_t* pHandle = (PWMC_R3_4_F3_Handle_t*)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  if (pHandle->_Super.RLDetectionMode == true) {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode (TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel (TIMx, LL_TIM_CHANNEL_CH1);

    if ((pHandle->pParams_str-> LowSideOutputs)== LS_PWM_TIMER)
      LL_TIM_CC_EnableChannel (TIMx, LL_TIM_CHANNEL_CH1N);
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
      LL_TIM_CC_DisableChannel (TIMx, LL_TIM_CHANNEL_CH1N);

    LL_TIM_OC_SetCompareCH1 (TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);

    /*  Channel2 configuration */
    LL_TIM_OC_SetMode (TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel (TIMx, LL_TIM_CHANNEL_CH2);

    if ((pHandle->pParams_str-> LowSideOutputs)== LS_PWM_TIMER)
      LL_TIM_CC_EnableChannel (TIMx, LL_TIM_CHANNEL_CH2N);
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
      LL_TIM_CC_DisableChannel (TIMx, LL_TIM_CHANNEL_CH2N);

    LL_TIM_OC_SetCompareCH2 (TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);

    /*  Channel3 configuration */
    LL_TIM_OC_SetMode (TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel (TIMx, LL_TIM_CHANNEL_CH3);

    if ((pHandle->pParams_str-> LowSideOutputs)== LS_PWM_TIMER)
      LL_TIM_CC_EnableChannel (TIMx, LL_TIM_CHANNEL_CH3N);
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
      LL_TIM_CC_DisableChannel (TIMx, LL_TIM_CHANNEL_CH3N);

    LL_TIM_OC_SetCompareCH3 (TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);

    pHandle->wADC_JSQR_phA = R3_4_F30X_ADC_InjectedChannelConfig (
      pHandle->pParams_str->ADCx_1, pHandle->pParams_str->bIaChannel, 1u,
      pHandle->pParams_str->b_IaSamplingTime, 1u, LL_ADC_INJ_TRIG_EXT_RISING,
      pHandle->ADC_ExternalTriggerInjected);
    pHandle->wADC_JSQR_phB = R3_4_F30X_ADC_InjectedChannelConfig (
      pHandle->pParams_str->ADCx_2, pHandle->pParams_str->bIbChannel, 1u,
      pHandle->pParams_str->b_IbSamplingTime, 1u, LL_ADC_INJ_TRIG_EXT_RISING,
      pHandle->ADC_ExternalTriggerInjected);

    pHandle->_Super.pFctGetPhaseCurrents = &R3_4_F30X_GetPhaseCurrents;
    pHandle->_Super.pFctTurnOnLowSides = &R3_4_F30X_TurnOnLowSides;
    pHandle->_Super.pFctSwitchOnPwm = &R3_4_F30X_SwitchOnPWM;
    pHandle->_Super.pFctSwitchOffPwm = &R3_4_F30X_SwitchOffPWM;

    pHandle->_Super.RLDetectionMode = false;
    }
  }
//}}}
//{{{
/**
 * @brief  It is used to set the PWM dutycycle for R/L detection.
 * @param  pHandle: handler of the current instance of the PWM component
 * @param  hDuty to be applied in uint16_t
 * @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR'
 *         otherwise. These error codes are defined in mc_type.h
 */
uint16_t R3_4_F30X_RLDetectionModeSetDuty (PWMC_Handle_t* pHdl, uint16_t hDuty) {

  PWMC_R3_4_F3_Handle_t *pHandle = (PWMC_R3_4_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  pR3_4_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;
  uint16_t hAux;

  uint32_t val = ((uint32_t)(pHandle->Half_PWMPeriod) * (uint32_t)(hDuty)) >> 16;
  pHandle->_Super.hCntPhA = (uint16_t)(val);

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM1;

  TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - pHandle->_Super.Ton;
  TIMx->CCR3 = pHandle->_Super.Toff;

  if (pDOPAMPParams_str) {
    pHandle->wOAMP1CR = pDOPAMPParams_str->wOPAMP_Selection->CSR;
    pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
    pHandle->wOAMP1CR |= pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHA;

    pHandle->wOAMP2CR = pDOPAMPParams_str->wOPAMP2_Selection->CSR;
    pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
    pHandle->wOAMP2CR |= pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHB;
    }
  else {
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
    pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phB;
    }

  TIMx->CCR1 = pHandle->_Super.hCntPhA;

  /* Switch Context */
  if (pDOPAMPParams_str) {
    pDOPAMPParams_str->wOPAMP_Selection->CSR = pHandle->wOAMP1CR;
    pDOPAMPParams_str->wOPAMP2_Selection->CSR = pHandle->wOAMP2CR;
    }
  pHandle->pParams_str->ADCx_1->JSQR = pHandle->wADC1_JSQR;
  pHandle->pParams_str->ADCx_2->JSQR = pHandle->wADC2_JSQR;

  /* Limit for update event */
  /* Check the status flag. If an update event has occurred before to set new
  values of regs the FOC rate is too high */
  if (LL_TIM_IsActiveFlag_UPDATE(TIMx))
    hAux = MC_FOC_DURATION;
  else
    hAux = MC_NO_ERROR;
  if (pHandle->_Super.SWerror == 1u) {
    hAux = MC_FOC_DURATION;
    pHandle->_Super.SWerror = 0u;
    }
  return hAux;
  }
//}}}
