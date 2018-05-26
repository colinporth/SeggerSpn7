// ihm07m1.c
#include "ihm07m1.h"
#include "6StepLib.h"

extern SIXSTEP_Base_InitTypeDef SIXSTEP_parameters;
extern SIXSTEP_PI_PARAM_InitTypeDef_t PI_parameters;

//{{{
static void L6230_ECH1CH2_DCH3_IO_Write() {

  HAL_GPIO_WritePin (GPIO_PORT_1, GPIO_CH1, GPIO_SET);      //EN1 ENABLE
  HAL_GPIO_WritePin (GPIO_PORT_1, GPIO_CH2, GPIO_SET);      //EN2 DISABLE
  HAL_GPIO_WritePin (GPIO_PORT_1, GPIO_CH3, GPIO_RESET);    //EN3 ENABLE
  }
//}}}
//{{{
static void L6230_ECH1CH3_DCH2_IO_Write()
{
  HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH1,GPIO_SET);    //EN1 ENABLE
  HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH2,GPIO_RESET);  //EN2 DISABLE
  HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH3,GPIO_SET);    //EN3 ENABLE
}
//}}}
//{{{
static void L6230_ECH2CH3_DCH1_IO_Write()
{
  HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH1,GPIO_RESET);  //EN1 DISABLE
  HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH2,GPIO_SET);    //EN2 ENABLE
  HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH3,GPIO_SET);    //EN3 ENABLE
}
//}}}
//{{{
static void L6230_DCH1CH2CH3_IO_Write()
{
  HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH1,GPIO_RESET);  //EN1 DISABLE
  HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH2,GPIO_RESET);  //EN2 DISABLE
  HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH3,GPIO_RESET);  //EN3 DISABLE
}
//}}}
//{{{
static void L6230_Start_PWM_generation() {

  HAL_TIM_PWM_Start (&HF_TIMx, HF_TIMx_CH1);  // TIM1_CH1 ENABLE
  HAL_TIM_PWM_Start (&HF_TIMx, HF_TIMx_CH2);  // TIM1_CH2 ENABLE
  HAL_TIM_PWM_Start (&HF_TIMx, HF_TIMx_CH3);  // TIM1_CH3 ENABLE
  }
//}}}
//{{{
static void L6230_Stop_PWM_generation() {

  HAL_TIM_PWM_Stop (&HF_TIMx, HF_TIMx_CH1);  // TIM1_CH1 DISABLE
  HAL_TIM_PWM_Stop (&HF_TIMx, HF_TIMx_CH2);  // TIM1_CH2 DISABLE
  HAL_TIM_PWM_Stop (&HF_TIMx, HF_TIMx_CH3);  // TIM1_CH3 DISABLE
  }
//}}}
//{{{
static void L6230_HFTIM_DC_CH1 (uint16_t CCRx) {
  HF_TIMx.Instance->HF_TIMx_CCR1 = CCRx;
  }
//}}}
//{{{
static void L6230_HFTIM_DC_CH2 (uint16_t CCRx) {
  HF_TIMx.Instance->HF_TIMx_CCR2 = CCRx;
  }
//}}}
//{{{
static void L6230_HFTIM_DC_CH3 (uint16_t CCRx) {
  HF_TIMx.Instance->HF_TIMx_CCR3 = CCRx;
  }
//}}}

static void EnableInput_CH1_E_CH2_E_CH3_D() { L6230_ECH1CH2_DCH3_IO_Write(); }
static void EnableInput_CH1_E_CH2_D_CH3_E() { L6230_ECH1CH3_DCH2_IO_Write(); }
static void EnableInput_CH1_D_CH2_E_CH3_E() { L6230_ECH2CH3_DCH1_IO_Write(); }
static void DisableInput_CH1_D_CH2_D_CH3_D() { L6230_DCH1CH2CH3_IO_Write(); }
static void Start_PWM_driving() { L6230_Start_PWM_generation(); }
static void Stop_PWM_driving() { L6230_Stop_PWM_generation(); }
static void HF_TIMx_SetDutyCycle_CH1 (uint16_t CCR_value) { L6230_HFTIM_DC_CH1 (CCR_value); }
static void HF_TIMx_SetDutyCycle_CH2 (uint16_t CCR_value) { L6230_HFTIM_DC_CH2 (CCR_value); }
static void HF_TIMx_SetDutyCycle_CH3 (uint16_t CCR_value) { L6230_HFTIM_DC_CH3 (CCR_value); }
static void Current_Reference_Start() { START_Ref_Generation(); }
static void Current_Reference_Stop() { STOP_Ref_Generation(); }
static void Current_Reference_Setvalue(uint16_t Iref) { Set_Ref_Generation (Iref); }
//{{{  struct L6230_MotorDriver_TypeDef
typedef struct {
  void (*EnableInput_CH1_E_CH2_E_CH3_D)(void);  /*!< Enable the channel 1,2 and Disable the channel 3 */
  void (*EnableInput_CH1_E_CH2_D_CH3_E)(void);  /*!< Enable the channel 1,3 and Disable the channel 2 */
  void (*EnableInput_CH1_D_CH2_E_CH3_E)(void);  /*!< Enable the channel 2,3 and Disable the channel 1 */
  void (*DisableInput_CH1_D_CH2_D_CH3_D)(void); /*!< Disable all channels */
  void (*Start_PWM_driving)(void);              /*!< Start PWM generation */
  void (*Stop_PWM_driving)(void);               /*!< Stop PWM generation */
  void (*HF_TIMx_SetDutyCycle_CH1)(uint16_t);   /*!< High Frequency Timer - Change DutyCycle value for CH1 */
  void (*HF_TIMx_SetDutyCycle_CH2)(uint16_t);   /*!< High Frequency Timer - Change DutyCycle value for CH2 */
  void (*HF_TIMx_SetDutyCycle_CH3)(uint16_t);   /*!< High Frequency Timer - Change DutyCycle value for CH3 */
  void (*Current_Reference_Start)(void);        /*!< Start current reference generation for closed loop control */
  void (*Current_Reference_Stop)(void);         /*!< Stop current reference generation for closed loop control */
  void (*Current_Reference_Setvalue)(uint16_t); /*!< Set current reference value for closed loop control */
  } L6230_MotorDriver_TypeDef;                          /*!< MC driver handler */
//}}}
//{{{
L6230_MotorDriver_TypeDef L6230MotorDriver = {
  EnableInput_CH1_E_CH2_E_CH3_D,
  EnableInput_CH1_E_CH2_D_CH3_E,
  EnableInput_CH1_D_CH2_E_CH3_E,
  DisableInput_CH1_D_CH2_D_CH3_D,
  Start_PWM_driving,
  Stop_PWM_driving,
  HF_TIMx_SetDutyCycle_CH1,
  HF_TIMx_SetDutyCycle_CH2,
  HF_TIMx_SetDutyCycle_CH3,
  Current_Reference_Start,
  Current_Reference_Stop,
  Current_Reference_Setvalue,
  };
//}}}

//{{{
void START_Ref_Generation() {
  REFx.Instance->CCR1 = 0;
  HAL_TIM_PWM_Start (&REFx, HF_TIMx_CH1);
  }
//}}}
//{{{
void STOP_Ref_Generation() {
  REFx.Instance->CCR1 = 0;
  HAL_TIM_PWM_Stop(&REFx, HF_TIMx_CH1);
  }
//}}}
//{{{
void Set_Ref_Generation (uint16_t Iref) {
  REFx.Instance->CCR1 = (uint32_t)(Iref * REFx.Instance->ARR)/4096;
  }
//}}}

//{{{
void START_DAC() {
  HAL_DAC_Start (&DACx,DACx_CH);
  }
//}}}
//{{{
void STOP_DAC() {
  HAL_DAC_Stop (&DACx,DACx_CH);
  }
//}}}
//{{{
void SET_DAC_value (uint16_t dac_value) {
  HAL_DAC_SetValue (&DACx, DACx_CH, DACx_ALIGN, dac_value);
  }
//}}}
//{{{
void Bemf_delay_calc() {

 if (PI_parameters.Reference >= 0) {
   if(SIXSTEP_parameters.speed_fdbk_filtered<=12000 && SIXSTEP_parameters.speed_fdbk_filtered>10000)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_1;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=10000 && SIXSTEP_parameters.speed_fdbk_filtered>7800)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_2;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=7800 && SIXSTEP_parameters.speed_fdbk_filtered>6400)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_3;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=6400 && SIXSTEP_parameters.speed_fdbk_filtered>5400)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_4;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=5400 && SIXSTEP_parameters.speed_fdbk_filtered>4650)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_5;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=4650 && SIXSTEP_parameters.speed_fdbk_filtered>4100)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_6;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=4100 && SIXSTEP_parameters.speed_fdbk_filtered>3650)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_7;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=3650 && SIXSTEP_parameters.speed_fdbk_filtered>3300)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_8;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=3300 && SIXSTEP_parameters.speed_fdbk_filtered>2600)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_9;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=2600 && SIXSTEP_parameters.speed_fdbk_filtered>1800)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_10;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=1800 && SIXSTEP_parameters.speed_fdbk_filtered>1500)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_11;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=1500 && SIXSTEP_parameters.speed_fdbk_filtered>1300)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_12;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=1300 && SIXSTEP_parameters.speed_fdbk_filtered>1000)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_13;
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=1000 && SIXSTEP_parameters.speed_fdbk_filtered>500)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_14;
     }
   else {
    if(SIXSTEP_parameters.speed_fdbk_filtered>=-12000 && SIXSTEP_parameters.speed_fdbk_filtered<-10000)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_1;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-10000 && SIXSTEP_parameters.speed_fdbk_filtered<-7800)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_2;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-7800 && SIXSTEP_parameters.speed_fdbk_filtered<-6400)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_3;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-6400 && SIXSTEP_parameters.speed_fdbk_filtered<-5400)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_4;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-5400 && SIXSTEP_parameters.speed_fdbk_filtered<-4650)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_5;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-4650 && SIXSTEP_parameters.speed_fdbk_filtered<-4100)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_6;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-4100 && SIXSTEP_parameters.speed_fdbk_filtered<-3650)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_7;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-3650 && SIXSTEP_parameters.speed_fdbk_filtered<-3300)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_8;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-3300 && SIXSTEP_parameters.speed_fdbk_filtered<-2650)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_9;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-2600 && SIXSTEP_parameters.speed_fdbk_filtered<-1800)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_10;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1800 && SIXSTEP_parameters.speed_fdbk_filtered<-1500)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_11;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1500 && SIXSTEP_parameters.speed_fdbk_filtered<-1300)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_12;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1300 && SIXSTEP_parameters.speed_fdbk_filtered<-1000)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_13;
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1000 && SIXSTEP_parameters.speed_fdbk_filtered<-500)
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_14;
    }
  }
//}}}

//{{{
void MC_SixStep_Nucleo_Init() {

  // TIM ETR CONFIGURATION
  TIM_ClearInputConfigTypeDef sClearInputConfig;
  sClearInputConfig.ClearInputState = 1;
  sClearInputConfig.ClearInputSource = TIM_CLEARINPUTSOURCE_ETR;
  sClearInputConfig.ClearInputPolarity = TIM_CLEARINPUTPOLARITY_NONINVERTED;
  sClearInputConfig.ClearInputPrescaler = TIM_CLEARINPUTPRESCALER_DIV1;
  sClearInputConfig.ClearInputFilter = 0;
  HAL_TIM_ConfigOCrefClear (&HF_TIMx, &sClearInputConfig, HF_TIMx_CH1);
  HAL_TIM_ConfigOCrefClear (&HF_TIMx, &sClearInputConfig, HF_TIMx_CH2);
  HAL_TIM_ConfigOCrefClear (&HF_TIMx, &sClearInputConfig, HF_TIMx_CH3);

  // stop TIM during Breakpoint
  __HAL_FREEZE_TIM1_DBGMCU();
  __HAL_TIM_ENABLE_IT (&htim1, TIM_IT_BREAK);

  // REGULAR CHANNELS CONFIGURATION
  // Current feedabck
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = ADC_CH_1;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_CH_1_ST;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel (&hadc1, &sConfig);

  // Bus voltage
  sConfig.Channel = ADC_CH_3;
  sConfig.SamplingTime = ADC_CH_3_ST;
  HAL_ADC_ConfigChannel (&hadc1, &sConfig);

  // Temperature feedback
  sConfig.Channel = ADC_CH_4;
  sConfig.SamplingTime = ADC_CH_4_ST;
  HAL_ADC_ConfigChannel (&hadc1, &sConfig);

  // BEMF feedback phase A
  sConfig.Channel = ADC_Bemf_CH1;
  sConfig.SamplingTime = ADC_Bemf_CH1_ST;
  HAL_ADC_ConfigChannel (&hadc1, &sConfig);

  // BEMF feedback phase B
  sConfig.Channel = ADC_Bemf_CH2;
  sConfig.SamplingTime = ADC_Bemf_CH2_ST;
  HAL_ADC_ConfigChannel (&hadc1, &sConfig);

  // BEMF feedback phase C
  sConfig.Channel = ADC_Bemf_CH3;
  sConfig.SamplingTime = ADC_Bemf_CH3_ST;
  HAL_ADC_ConfigChannel (&hadc1, &sConfig);

  // Potentiometer
  sConfig.Channel = ADC_CH_2;
  sConfig.SamplingTime = ADC_CH_2_ST;
  HAL_ADC_ConfigChannel (&hadc1, &sConfig);
  }
//}}}
//{{{
void MC_SixStep_ADC_Channel (uint32_t adc_ch) {

  ADCx.Instance->CR |= ADC_CR_ADSTP;
  while (ADCx.Instance->CR & ADC_CR_ADSTP);

  // Clear the old SQx bits for the selected rank
  ADCx.Instance->SQR1 &= ~__HAL_ADC_SQR1_RK (ADC_SQR2_SQ5, 1);

  // Set the SQx bits for the selected rank
  ADCx.Instance->SQR1 |= __HAL_ADC_SQR1_RK (adc_ch, 1);
  ADCx.Instance->CR |= ADC_CR_ADSTART;
  }
//}}}

//{{{
void MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D() {
  L6230MotorDriver.EnableInput_CH1_E_CH2_E_CH3_D();
  }
//}}}
//{{{
void MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E() {
  L6230MotorDriver.EnableInput_CH1_E_CH2_D_CH3_E();
  }
//}}}
//{{{
void MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E() {
  L6230MotorDriver.EnableInput_CH1_D_CH2_E_CH3_E();
  }
//}}}
//{{{
void MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D() {
  L6230MotorDriver.DisableInput_CH1_D_CH2_D_CH3_D();
  }
//}}}

//{{{
void MC_SixStep_Start_PWM_driving() {
  L6230MotorDriver.Start_PWM_driving();
  }
//}}}
//{{{
void MC_SixStep_Stop_PWM_driving() {
  L6230MotorDriver.Stop_PWM_driving();
  }
//}}}

//{{{
void MC_SixStep_HF_TIMx_SetDutyCycle_CH1 (uint16_t CCR_value) {
  L6230MotorDriver.HF_TIMx_SetDutyCycle_CH1 (CCR_value);
  }
//}}}
//{{{
void MC_SixStep_HF_TIMx_SetDutyCycle_CH2 (uint16_t CCR_value) {
  L6230MotorDriver.HF_TIMx_SetDutyCycle_CH2 (CCR_value);
  }
//}}}
//{{{
void MC_SixStep_HF_TIMx_SetDutyCycle_CH3 (uint16_t CCR_value) {
  L6230MotorDriver.HF_TIMx_SetDutyCycle_CH3 (CCR_value);
  }
//}}}

//{{{
void MC_SixStep_Current_Reference_Start() {
  L6230MotorDriver.Current_Reference_Start();
  }
//}}}
//{{{
void MC_SixStep_Current_Reference_Stop() {
  L6230MotorDriver.Current_Reference_Stop();
  }
//}}}
//{{{
void MC_SixStep_Current_Reference_Setvalue (uint16_t Iref) {
  L6230MotorDriver.Current_Reference_Setvalue (Iref);
  }
//}}}

//{{{
void BSP_X_NUCLEO_FAULT_LED_ON() {
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  }
//}}}
//{{{
void BSP_X_NUCLEO_FAULT_LED_OFF() {
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  }
//}}}
