// 6StepLib.c
#include "6StepLib.h"

SIXSTEP_Base_InitTypeDef SIXSTEP_parameters;   // Main SixStep structure
SIXSTEP_PI_PARAM_InitTypeDef_t PI_parameters;  // SixStep PI regulator structure
//{{{  vars
uint16_t Rotor_poles_pairs;               //  Number of pole pairs of the motor */
uint32_t mech_accel_hz = 0;               //  Hz -- Mechanical acceleration rate */
uint32_t constant_k = 0;                  //  1/3*mech_accel_hz */
uint32_t Time_vector_tmp = 0;             //  Startup variable  */
uint32_t Time_vector_prev_tmp = 0 ;       //  Startup variable  */
uint32_t T_single_step = 0;               //  Startup variable  */
uint32_t T_single_step_first_value = 0;   //  Startup variable  */
int32_t  delta = 0;                       //  Startup variable  */
uint16_t index_array = 1;                 //  Speed filter variable */
int16_t speed_tmp_array[FILTER_DEEP];     //  Speed filter variable */
uint16_t speed_tmp_buffer[FILTER_DEEP];   //  Potentiometer filter variable */
uint16_t HFBuffer[HFBUFFERSIZE];          //  Buffer for Potentiometer Value Filtering at the High-Frequency ADC conversion */
uint16_t HFBufferIndex = 0;               //  High-Frequency Buffer Index */
uint8_t  array_completed = FALSE;         //  Speed filter variable */
uint8_t  buffer_completed = FALSE;        //  Potentiometer filter variable */
uint8_t  UART_FLAG_RECEIVE = FALSE;       //  UART commmunication flag */
uint32_t ARR_LF = 0;                      //  Autoreload LF TIM variable */
int32_t Mech_Speed_RPM = 0;               //  Mechanical motor speed */
int32_t El_Speed_Hz = 0;                  //  Electrical motor speed */
uint16_t index_adc_chn = 0;               //  Index of ADC channel selector for measuring */
uint16_t index_motor_run = 0;             //  Tmp variable for DEMO mode */
uint16_t test_motor_run = 1;              //  Tmp variable for DEMO mode */
uint8_t Enable_start_button = TRUE;       //  Start/stop button filter to avoid double command */
uint16_t index_ARR_step = 1;
uint32_t n_zcr_startup = 0;
uint16_t index_startup_motor = 1;
uint16_t target_speed = TARGET_SPEED;     //  Target speed for closed loop control */
uint16_t shift_n_sqrt = 14;
uint16_t cnt_bemf_event = 0;
uint8_t startup_bemf_failure = 0;
uint8_t speed_fdbk_error = 0;
uint8_t dac_status = DAC_ENABLE;
uint16_t index_align = 1;
int32_t speed_sum_sp_filt = 0;
int32_t speed_sum_pot_filt = 0;
uint16_t index_pot_filt = 1;
int16_t potent_filtered = 0;
uint32_t Tick_cnt = 0;
uint32_t counter_ARR_Bemf = 0;
uint64_t constant_multiplier_tmp = 0;
//}}}

//{{{
static uint64_t MCM_Sqrt (uint64_t wInput) {


  uint64_t wtemproot;
  if (wInput <= (uint64_t)((uint64_t)2097152 << shift_n_sqrt))
    wtemproot = (uint64_t)((uint64_t)128 << shift_n_sqrt);
  else
    wtemproot = (uint64_t)((uint64_t)8192 << shift_n_sqrt);

  uint8_t biter = 0u;
  uint64_t wtemprootnew;
  do {
    wtemprootnew = (wtemproot + wInput/wtemproot)>>1;
    if (wtemprootnew == wtemproot)
      biter = (shift_n_sqrt-1);
    else {
      biter ++;
      wtemproot = wtemprootnew;
      }
    } while (biter < (shift_n_sqrt-1));

  return (wtemprootnew);
  }
//}}}

//{{{
static int32_t MC_GetElSpeedHz() {

  if (__HAL_TIM_GetAutoreload (&LF_TIMx) != 0xFFFF) {
    uint16_t prsc = LF_TIMx.Instance->PSC;
    El_Speed_Hz = (int32_t)((SIXSTEP_parameters.SYSCLK_frequency) / (prsc)) /
                            (__HAL_TIM_GetAutoreload (&LF_TIMx) * 6);
    }
  else
    El_Speed_Hz = 0;

  if (PI_parameters.Reference < 0)
    return (-El_Speed_Hz);
  else
    return (El_Speed_Hz);
  }
//}}}
//{{{
static int32_t MC_GetMechSpeedRPM() {

  Mech_Speed_RPM = (int32_t)(MC_GetElSpeedHz() *  60 / Rotor_poles_pairs);
  return (Mech_Speed_RPM);
  }
//}}}
//{{{
static uint16_t MC_Potentiometer_filter (uint16_t potentiometer_value) {

  if (buffer_completed == FALSE) {
    speed_tmp_buffer[index_pot_filt] = potentiometer_value;
    speed_sum_pot_filt = 0;
    for (uint16_t i = 1; i <= index_pot_filt;i++)
      speed_sum_pot_filt = speed_sum_pot_filt + speed_tmp_buffer[i];
    potent_filtered = speed_sum_pot_filt/index_pot_filt;
    index_pot_filt++;

    if (index_pot_filt >= FILTER_DEEP) {
      index_pot_filt = 1;
      buffer_completed = TRUE;
       }
    }

  else {
     index_pot_filt++;
     if (index_pot_filt >= FILTER_DEEP)
       index_pot_filt = 1;

     speed_sum_pot_filt = 0;
     speed_tmp_buffer[index_pot_filt] = potentiometer_value;
     uint16_t speed_max = 0;
     for (uint16_t i = 1; i < FILTER_DEEP;i++) {
       uint16_t val = speed_tmp_buffer[i];
       if (val > speed_max)
         speed_max = val;
       speed_sum_pot_filt += val;
       }
     speed_sum_pot_filt -= speed_max;
     potent_filtered = speed_sum_pot_filt/(FILTER_DEEP-2);
    }

  if (potent_filtered==0)
    potent_filtered = 1;

  return(potent_filtered);
  }
//}}}
//{{{
static void MC_SixStep_Speed_Val_target_potentiometer() {

  target_speed = SIXSTEP_parameters.ADC_Regular_Buffer[1] * MAX_POT_SPEED/ 4096;

  if (target_speed < MIN_POT_SPEED)
    target_speed = MIN_POT_SPEED;

  if (target_speed > (MAX_POT_SPEED/VAL_POT_SPEED_DIV))
    target_speed = (MAX_POT_SPEED/VAL_POT_SPEED_DIV);
  }
//}}}
//{{{
static void MC_SixStep_Speed_Potentiometer() {

  uint16_t i=0;
  uint32_t sum = 0;
  uint16_t mean = 0;
  uint16_t max = 0;
  for (i = 0; i < HFBUFFERSIZE; i++) {
    uint16_t val = HFBuffer[i];
    sum += val;
    if (val > max)
      max = val;
    }

  sum -= max;
  mean = sum / (HFBUFFERSIZE - 1);

  SIXSTEP_parameters.Speed_Ref_filtered = MC_Potentiometer_filter(mean);
  }
//}}}

//{{{
static void MC_SixStep_TABLE (uint8_t step_number) {

  if (GPIO_COMM == 1)
    HAL_GPIO_TogglePin (GPIO_PORT_COMM, GPIO_CH_COMM);

  switch (step_number) {
    case 1:
      MC_SixStep_HF_TIMx_SetDutyCycle_CH1 (SIXSTEP_parameters.pulse_value);
      MC_SixStep_HF_TIMx_SetDutyCycle_CH2 (0);
      MC_SixStep_HF_TIMx_SetDutyCycle_CH3 (0);
      MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D();
      SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[3];
      break;

    case 2:
      MC_SixStep_HF_TIMx_SetDutyCycle_CH1 (SIXSTEP_parameters.pulse_value);
      MC_SixStep_HF_TIMx_SetDutyCycle_CH2 (0);
      MC_SixStep_HF_TIMx_SetDutyCycle_CH3 (0);
      MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E();
      SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[2];
      break;

    case 3:
      MC_SixStep_HF_TIMx_SetDutyCycle_CH2 (SIXSTEP_parameters.pulse_value);
      MC_SixStep_HF_TIMx_SetDutyCycle_CH3 (0);
      MC_SixStep_HF_TIMx_SetDutyCycle_CH1 (0);
      MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E();
      SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[1];
      break;

    case 4:
      MC_SixStep_HF_TIMx_SetDutyCycle_CH2 (SIXSTEP_parameters.pulse_value);
      MC_SixStep_HF_TIMx_SetDutyCycle_CH1 (0);
      MC_SixStep_HF_TIMx_SetDutyCycle_CH3 (0);
      MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D();
      SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[3];
     break;

    case 5:
      MC_SixStep_HF_TIMx_SetDutyCycle_CH3 (SIXSTEP_parameters.pulse_value);
      MC_SixStep_HF_TIMx_SetDutyCycle_CH1 (0);
      MC_SixStep_HF_TIMx_SetDutyCycle_CH2 (0);
      MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E();
      SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[2];
      break;

    case 6:
      MC_SixStep_HF_TIMx_SetDutyCycle_CH3 (SIXSTEP_parameters.pulse_value);
      MC_SixStep_HF_TIMx_SetDutyCycle_CH2 (0);
      MC_SixStep_HF_TIMx_SetDutyCycle_CH1 (0);
      MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E();
      SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[1];
      break;
     }
  }
//}}}
//{{{
static void MC_SixStep_NEXT_step() {

  if (SIXSTEP_parameters.CMD == TRUE) {
    SIXSTEP_parameters.CMD = FALSE;
    MC_SixStep_Start_PWM_driving();
    }
  ARR_LF = __HAL_TIM_GetAutoreload (&LF_TIMx);

  if (SIXSTEP_parameters.ALIGN_OK == TRUE) {
    SIXSTEP_parameters.speed_fdbk = MC_GetMechSpeedRPM();
    SIXSTEP_parameters.demagn_counter = 1;
    if (SIXSTEP_parameters.status_prev != SIXSTEP_parameters.step_position)
      n_zcr_startup = 0;

    if (PI_parameters.Reference>=0) {
      SIXSTEP_parameters.step_position++;
      if (SIXSTEP_parameters.CL_READY == TRUE)
        SIXSTEP_parameters.VALIDATION_OK = TRUE;
      if (SIXSTEP_parameters.step_position > 6)
        SIXSTEP_parameters.step_position = 1;
      }
    else {
      SIXSTEP_parameters.step_position--;
      if (SIXSTEP_parameters.CL_READY == TRUE)
        SIXSTEP_parameters.VALIDATION_OK = TRUE;
      if (SIXSTEP_parameters.step_position < 1)
        SIXSTEP_parameters.step_position = 6;
      }
    }

  if (SIXSTEP_parameters.VALIDATION_OK == 1) {
    // Motor Stall condition detection and Speed-Feedback error generation
    SIXSTEP_parameters.BEMF_Tdown_count++;
    if (SIXSTEP_parameters.BEMF_Tdown_count>BEMF_CONSEC_DOWN_MAX)
      speed_fdbk_error = 1;
    else
      __HAL_TIM_SetAutoreload (&LF_TIMx,0xFFFF);
    }
  MC_SixStep_TABLE (SIXSTEP_parameters.step_position);

  // It controls if the changing step request appears during DOWNcounting
  // in this case it changes the ADC channel UP-COUNTING direction started  DIR = 0*/
  if (__HAL_TIM_DIRECTION_STATUS (&HF_TIMx)) {
    switch (SIXSTEP_parameters.step_position) {
      case 1: SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[3]; break;
      case 2: SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[2]; break;
      case 3: SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[1]; break;
      case 4: SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[3]; break;
      case 5: SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[2]; break;
      case 6: SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[1]; break;
      }

    MC_SixStep_ADC_Channel(SIXSTEP_parameters.CurrentRegular_BEMF_ch);
    }
  }
//}}}
//{{{
static void MC_SixStep_Ramp_Motor_calc() {

  uint32_t constant_multiplier = 100;
  uint32_t constant_multiplier_2 = 4000000000;

  if (index_startup_motor == 1) {
    mech_accel_hz = SIXSTEP_parameters.ACCEL * Rotor_poles_pairs / 60;
    constant_multiplier_tmp = (uint64_t)constant_multiplier * (uint64_t)constant_multiplier_2;
    constant_k = constant_multiplier_tmp / (3 * mech_accel_hz);
    MC_SixStep_Current_Reference_Setvalue (SIXSTEP_parameters.Ireference);
    Time_vector_prev_tmp = 0;
    }

  if (index_startup_motor < NUMBER_OF_STEPS) {
    Time_vector_tmp = ((uint64_t) 1000 * (uint64_t)1000 * (uint64_t) MCM_Sqrt(((uint64_t)index_startup_motor * (uint64_t)constant_k)))/632455;
    delta = Time_vector_tmp - Time_vector_prev_tmp;
    if (index_startup_motor==1) {
      T_single_step_first_value = (2 * 3141) * delta / 1000;
      SIXSTEP_parameters.ARR_value = (uint32_t)(65535);
      }
    else {
      T_single_step = (2 * 3141)*delta/1000;
      SIXSTEP_parameters.ARR_value = (uint32_t)(65535 * T_single_step) / (T_single_step_first_value);
      }
    }
  else
    index_startup_motor = 1;

  if (index_startup_motor==1)
    SIXSTEP_parameters.prescaler_value = (((SIXSTEP_parameters.SYSCLK_frequency / 1000000) * T_single_step_first_value)/65535) - 1;
  if (SIXSTEP_parameters.STATUS != ALIGNMENT && SIXSTEP_parameters.STATUS != START)
    index_startup_motor++;
  else
    Time_vector_tmp = 0;

  Time_vector_prev_tmp =  Time_vector_tmp;
  }
//}}}
//{{{
static void MC_SixStep_ARR_step() {

  if (SIXSTEP_parameters.ALIGNMENT == FALSE)
    SIXSTEP_parameters.ALIGNMENT = TRUE;

  if (SIXSTEP_parameters.ALIGN_OK == TRUE) {
    if (PI_parameters.Reference >= 0) {
      if (SIXSTEP_parameters.VALIDATION_OK != TRUE) {
        SIXSTEP_parameters.STATUS = STARTUP;
        MC_SixStep_Ramp_Motor_calc();
        if (index_ARR_step < SIXSTEP_parameters.numberofitemArr) {
          LF_TIMx.Init.Period = SIXSTEP_parameters.ARR_value;
          LF_TIMx.Instance->ARR = (uint32_t)LF_TIMx.Init.Period;
          index_ARR_step++;
          }
        else if (SIXSTEP_parameters.ARR_OK == 0) {
          index_ARR_step = 1;
          SIXSTEP_parameters.ACCEL>>=1;
          if (SIXSTEP_parameters.ACCEL < MINIMUM_ACC)
            SIXSTEP_parameters.ACCEL = MINIMUM_ACC;
          MC_StopMotor();
          SIXSTEP_parameters.STATUS = STARTUP_FAILURE;
          }
        }
      else {
        SIXSTEP_parameters.ARR_OK = 1;
        index_startup_motor = 1;
        index_ARR_step = 1;
        }
      }
    else {
      if (SIXSTEP_parameters.VALIDATION_OK != TRUE) {
        SIXSTEP_parameters.STATUS = STARTUP;
        MC_SixStep_Ramp_Motor_calc();
        if (index_ARR_step < SIXSTEP_parameters.numberofitemArr) {
          LF_TIMx.Init.Period = SIXSTEP_parameters.ARR_value;
          LF_TIMx.Instance->ARR = (uint32_t)LF_TIMx.Init.Period;
          index_ARR_step++;
          }
        else if(SIXSTEP_parameters.ARR_OK==0) {
          index_ARR_step = 1;
          SIXSTEP_parameters.ACCEL>>=1;
          if (SIXSTEP_parameters.ACCEL < MINIMUM_ACC)
            SIXSTEP_parameters.ACCEL = MINIMUM_ACC;
          MC_StopMotor();
          SIXSTEP_parameters.STATUS = STARTUP_FAILURE;
          }
        }
      else {
        SIXSTEP_parameters.ARR_OK = 1;
        index_startup_motor = 1;
        index_ARR_step = 1;
        }
      }
    }
  }
//}}}
//{{{
static void MC_SixStep_Alignment() {

  SIXSTEP_parameters.step_position = 6;
  LF_TIMx.Init.Period = SIXSTEP_parameters.ARR_value;
  LF_TIMx.Instance->ARR = (uint32_t)LF_TIMx.Init.Period;
  SIXSTEP_parameters.STATUS = ALIGNMENT;

  MC_SixStep_Speed_Val_target_potentiometer();

  index_align++;
  if (index_align >= TIME_FOR_ALIGN + 1) {
    SIXSTEP_parameters.ALIGN_OK = TRUE;
    SIXSTEP_parameters.STATUS = STARTUP;
    index_startup_motor = 1;
    MC_SixStep_Ramp_Motor_calc();
    LF_TIMx.Init.Prescaler = SIXSTEP_parameters.prescaler_value;
    LF_TIMx.Instance->PSC = LF_TIMx.Init.Prescaler;
    index_align = 0;
    }
  }
//}}}

//{{{
static void MC_Set_PI_param (SIXSTEP_PI_PARAM_InitTypeDef_t *PI_PARAM) {

  if (SIXSTEP_parameters.CW_CCW == 0)
    PI_PARAM->Reference = target_speed;
  else
    PI_PARAM->Reference = -target_speed;

  PI_PARAM->Kp_Gain = SIXSTEP_parameters.KP;
  PI_PARAM->Ki_Gain = SIXSTEP_parameters.KI;

  PI_PARAM->Lower_Limit_Output = LOWER_OUT_LIMIT;
  PI_PARAM->Upper_Limit_Output = UPPER_OUT_LIMIT;
  PI_PARAM->Max_PID_Output = FALSE;
  PI_PARAM->Min_PID_Output = FALSE;
  }
//}}}
//{{{
static int16_t MC_PI_Controller (SIXSTEP_PI_PARAM_InitTypeDef_t *PI_PARAM, int16_t speed_fdb) {

  int32_t Error = (PI_PARAM->Reference - speed_fdb);

  // Proportional term computation
  int32_t wProportional_Term = PI_PARAM->Kp_Gain * Error;

  // Integral term computation
  int32_t wIntegral_Term = 0;
  int32_t wIntegral_sum_temp = 0;
  if (PI_PARAM->Ki_Gain == 0)
    SIXSTEP_parameters.Integral_Term_sum = 0;
  else {
    wIntegral_Term = PI_PARAM->Ki_Gain * Error;
    wIntegral_sum_temp = SIXSTEP_parameters.Integral_Term_sum + wIntegral_Term;
    SIXSTEP_parameters.Integral_Term_sum = wIntegral_sum_temp;
    }

  if (SIXSTEP_parameters.Integral_Term_sum> KI_DIV * PI_PARAM->Upper_Limit_Output)
    SIXSTEP_parameters.Integral_Term_sum = KI_DIV* PI_PARAM->Upper_Limit_Output;

  if (SIXSTEP_parameters.Integral_Term_sum<-KI_DIV* PI_PARAM->Upper_Limit_Output)
    SIXSTEP_parameters.Integral_Term_sum = -KI_DIV* PI_PARAM->Upper_Limit_Output;

  // WARNING: the below instruction is not MISRA compliant, user should verify
  //          that Cortex-M3 assembly instruction ASR (arithmetic shift right)
  //          is used by the compiler to perform the shifts (instead of LSR logical shift right)*/
  int32_t wOutput_32 = (wProportional_Term/KP_DIV) + (SIXSTEP_parameters.Integral_Term_sum/KI_DIV);

  if (PI_PARAM->Reference > 0) {
    if (wOutput_32 > PI_PARAM->Upper_Limit_Output)
      wOutput_32 = PI_PARAM->Upper_Limit_Output;
    else if (wOutput_32 < PI_PARAM->Lower_Limit_Output)
      wOutput_32 = PI_PARAM->Lower_Limit_Output;
    }
  else {
    if (wOutput_32 < (- PI_PARAM->Upper_Limit_Output) )
      wOutput_32 = - (PI_PARAM->Upper_Limit_Output);
    else if (wOutput_32 > (-PI_PARAM->Lower_Limit_Output))
      wOutput_32 = (-PI_PARAM->Lower_Limit_Output);
    }

  return ((int16_t)(wOutput_32));
  }
//}}}

//{{{
static void MC_Speed_Filter() {

  if (array_completed == FALSE) {
    speed_tmp_array[index_array] = SIXSTEP_parameters.speed_fdbk;
    speed_sum_sp_filt = 0;
    for (uint16_t i = 1; i <= index_array;i++)
      speed_sum_sp_filt = speed_sum_sp_filt + speed_tmp_array[i];
    SIXSTEP_parameters.speed_fdbk_filtered = speed_sum_sp_filt/index_array;
    index_array++;

    if (index_array >= FILTER_DEEP) {
     index_array = 1;
     array_completed = TRUE;
     }
    }

  else {
    index_array++;
    if (index_array >= FILTER_DEEP)
      index_array = 1;

    speed_sum_sp_filt = 0;
    speed_tmp_array[index_array] = SIXSTEP_parameters.speed_fdbk;
    for (uint16_t i = 1; i < FILTER_DEEP;i++)
      speed_sum_sp_filt = speed_sum_sp_filt + speed_tmp_array[i];
    SIXSTEP_parameters.speed_fdbk_filtered = speed_sum_sp_filt/(FILTER_DEEP-1);
    }
  }
//}}}
//{{{
static void MC_Bemf_Delay() {
  Bemf_delay_calc();
  }
//}}}
//{{{
static void MC_Task_Speed() {

  if (dac_status == TRUE)
    SET_DAC_value (SIXSTEP_parameters.speed_fdbk_filtered);

  if ((SIXSTEP_parameters.speed_fdbk_filtered > (target_speed) ||
       SIXSTEP_parameters.speed_fdbk_filtered < (-target_speed)) &&
       SIXSTEP_parameters.VALIDATION_OK != TRUE) {
    SIXSTEP_parameters.STATUS = VALIDATION;
    SIXSTEP_parameters.SPEED_VALIDATED = TRUE;
    }

  if (SIXSTEP_parameters.SPEED_VALIDATED == TRUE &&
      SIXSTEP_parameters.BEMF_OK == TRUE &&
      SIXSTEP_parameters.CL_READY != TRUE)
    SIXSTEP_parameters.CL_READY = TRUE;

  if (SIXSTEP_parameters.VALIDATION_OK == TRUE) {
    SIXSTEP_parameters.STATUS = RUN;
    if (PI_parameters.Reference >= 0)
      SIXSTEP_parameters.Current_Reference = (uint16_t)MC_PI_Controller (&PI_parameters,(int16_t)SIXSTEP_parameters.speed_fdbk_filtered);
    else
      SIXSTEP_parameters.Current_Reference = (uint16_t)(-MC_PI_Controller (&PI_parameters,(int16_t)SIXSTEP_parameters.speed_fdbk_filtered));

    MC_SixStep_Current_Reference_Setvalue (SIXSTEP_parameters.Current_Reference);
    }

  MC_Bemf_Delay();
  }
//}}}

// callback interface
//{{{
void MC_SixStep_ARR_Bemf (uint8_t up_bemf) {

  if (SIXSTEP_parameters.status_prev != SIXSTEP_parameters.step_position) {
    if (SIXSTEP_parameters.SPEED_VALIDATED == TRUE) {
      if (GPIO_ZERO_CROSS == 1)
        HAL_GPIO_TogglePin(GPIO_PORT_ZCR,GPIO_CH_ZCR);
      if (cnt_bemf_event> BEMF_CNT_EVENT_MAX)
        startup_bemf_failure = 1;

      if (up_bemf == 1 && SIXSTEP_parameters.BEMF_OK !=TRUE) {
        n_zcr_startup++;
        cnt_bemf_event = 0;
        }
      else if(SIXSTEP_parameters.BEMF_OK !=TRUE)
        cnt_bemf_event++;

      if (n_zcr_startup>= NUMBER_ZCR && SIXSTEP_parameters.BEMF_OK !=TRUE ) {
        SIXSTEP_parameters.BEMF_OK = TRUE;
        n_zcr_startup = 0;
        }
      }

    SIXSTEP_parameters.status_prev = SIXSTEP_parameters.step_position;

    if (SIXSTEP_parameters.VALIDATION_OK == 1) {
      counter_ARR_Bemf = __HAL_TIM_GetCounter(&LF_TIMx);
      __HAL_TIM_SetAutoreload(&LF_TIMx,(counter_ARR_Bemf+ARR_LF/2));
      }
    }
  }
//}}}
//{{{
void MC_ADCx_SixStep_Bemf() {

  if (__HAL_TIM_DIRECTION_STATUS (&HF_TIMx)) {
    HAL_GPIO_WritePin (GPIO_PORT_COMM,GPIO_CH_COMM,GPIO_PIN_SET);
    // UP-counting direction started, GET the ADC value (PHASE CURRENT)*/
    if (SIXSTEP_parameters.STATUS != START && SIXSTEP_parameters.STATUS != ALIGNMENT) {
      switch (SIXSTEP_parameters.step_position) {
        //{{{
        case 6: {
          if (SIXSTEP_parameters.demagn_counter >= SIXSTEP_parameters.demagn_value) {
            SIXSTEP_parameters.ADC_BUFFER[1] = HAL_ADC_GetValue(&ADCx);
            if (PI_parameters.Reference>=0) {
             if (SIXSTEP_parameters.ADC_BUFFER[1]> SIXSTEP_parameters.ADC_BEMF_threshold_UP) {
               MC_SixStep_ARR_Bemf (1);
               SIXSTEP_parameters.BEMF_Tdown_count = 0;
               }
             }
           else {
             if (SIXSTEP_parameters.ADC_BUFFER[1]< SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
               MC_SixStep_ARR_Bemf (0);
             }
           }
         else
           SIXSTEP_parameters.demagn_counter++;
          }
          break;
        //}}}
        //{{{
        case 3: {
          if (SIXSTEP_parameters.demagn_counter >= SIXSTEP_parameters.demagn_value) {
            SIXSTEP_parameters.ADC_BUFFER[1] = HAL_ADC_GetValue(&ADCx);
            if (PI_parameters.Reference>=0) {
              if (SIXSTEP_parameters.ADC_BUFFER[1]< SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
                MC_SixStep_ARR_Bemf (0);
              }
            else {
              if (SIXSTEP_parameters.ADC_BUFFER[1]> SIXSTEP_parameters.ADC_BEMF_threshold_UP) {
                MC_SixStep_ARR_Bemf (1);
                SIXSTEP_parameters.BEMF_Tdown_count = 0;
                }
              }
            }
          else
            SIXSTEP_parameters.demagn_counter++;
          }
          break;
        //}}}
        //{{{
        case 5: {
          if (SIXSTEP_parameters.demagn_counter >= SIXSTEP_parameters.demagn_value) {
           SIXSTEP_parameters.ADC_BUFFER[2] = HAL_ADC_GetValue(&ADCx);
           if (PI_parameters.Reference>=0) {
             if (SIXSTEP_parameters.ADC_BUFFER[2]< SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
               MC_SixStep_ARR_Bemf (0);
             }
           else {
             if (SIXSTEP_parameters.ADC_BUFFER[2]> SIXSTEP_parameters.ADC_BEMF_threshold_UP) {
               MC_SixStep_ARR_Bemf (1);
               SIXSTEP_parameters.BEMF_Tdown_count = 0;
               }
             }
           }
         else
           SIXSTEP_parameters.demagn_counter++;
         }
         break;
        //}}}
        //{{{
        case 2: {
          if (SIXSTEP_parameters.demagn_counter >= SIXSTEP_parameters.demagn_value) {
            SIXSTEP_parameters.ADC_BUFFER[2] = HAL_ADC_GetValue(&ADCx);
            if (PI_parameters.Reference>=0) {
              if (SIXSTEP_parameters.ADC_BUFFER[2] > SIXSTEP_parameters.ADC_BEMF_threshold_UP) {
                MC_SixStep_ARR_Bemf (1);
                SIXSTEP_parameters.BEMF_Tdown_count = 0;
                }
              }
            else {
              if (SIXSTEP_parameters.ADC_BUFFER[2] < SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
                MC_SixStep_ARR_Bemf (0);
              }
            }
          else
           SIXSTEP_parameters.demagn_counter++;
          }
          break;
        //}}}
        //{{{
        case 4: {
         if (SIXSTEP_parameters.demagn_counter >= SIXSTEP_parameters.demagn_value) {
           SIXSTEP_parameters.ADC_BUFFER[3] = HAL_ADC_GetValue(&ADCx);
           if (PI_parameters.Reference>=0) {
             if (SIXSTEP_parameters.ADC_BUFFER[3] > SIXSTEP_parameters.ADC_BEMF_threshold_UP) {
               MC_SixStep_ARR_Bemf (1);
               SIXSTEP_parameters.BEMF_Tdown_count = 0;
               }
             }
           else {
             if (SIXSTEP_parameters.ADC_BUFFER[3] < SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
               MC_SixStep_ARR_Bemf (0);
             }
           }
         else
           SIXSTEP_parameters.demagn_counter++;
         }
         break;
        //}}}
        //{{{
        case 1: {
          if (SIXSTEP_parameters.demagn_counter >= SIXSTEP_parameters.demagn_value) {
            SIXSTEP_parameters.ADC_BUFFER[3] = HAL_ADC_GetValue(&ADCx);
            if (PI_parameters.Reference>=0) {
              if (SIXSTEP_parameters.ADC_BUFFER[3] < SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
                MC_SixStep_ARR_Bemf (0);
              }
            else {
              if (SIXSTEP_parameters.ADC_BUFFER[3] > SIXSTEP_parameters.ADC_BEMF_threshold_UP) {
                MC_SixStep_ARR_Bemf (1);
                SIXSTEP_parameters.BEMF_Tdown_count = 0;
                }
              }
            }
          else
            SIXSTEP_parameters.demagn_counter++;
          }
        break;
        //}}}
        }
      }

    // SET ADC CHANNEL FOR SPEED/CURRENT/VBUS *******************/
    // Set the channel for next ADC Regular reading */
    MC_SixStep_ADC_Channel(SIXSTEP_parameters.ADC_SEQ_CHANNEL[index_adc_chn]);
    HAL_GPIO_WritePin(GPIO_PORT_COMM,GPIO_CH_COMM,GPIO_PIN_RESET);
    }

  else {
    // Down-counting direction started, Set the channel for next ADC Regular reading
    SIXSTEP_parameters.ADC_Regular_Buffer[index_adc_chn] = HAL_ADC_GetValue (&ADCx);
    if (index_adc_chn == 1) {
      HFBuffer[HFBufferIndex++] = HAL_ADC_GetValue(&ADCx);
      if (HFBufferIndex >= HFBUFFERSIZE)
        HFBufferIndex = 0;
      }

    index_adc_chn++;
    if (index_adc_chn>3) index_adc_chn = 0;
      MC_SixStep_ADC_Channel (SIXSTEP_parameters.CurrentRegular_BEMF_ch);
    }
  }
//}}}
//{{{
void MC_TIMx_SixStep_timebase() {

  MC_SixStep_NEXT_step();

  // BASE TIMER - ARR modification for STEP frequency changing
  if (SIXSTEP_parameters.ARR_OK == 0)
    MC_SixStep_ARR_step();

  MC_Speed_Filter();
  }
//}}}
//{{{
static void MC_SysTick_SixStep_MediumFrequencyTask() {

  if ((SIXSTEP_parameters.ALIGNMENT == TRUE) && (SIXSTEP_parameters.ALIGN_OK == FALSE)) {
    printf ("align\n");
    MC_SixStep_Alignment();
    }

  #ifdef UART_COMM
    if (UART_FLAG_RECEIVE == TRUE)
      UART_Communication_Task();
  #endif


  if (SIXSTEP_parameters.VALIDATION_OK == TRUE && SIXSTEP_parameters.Potentiometer  == TRUE)
    MC_SixStep_Speed_Potentiometer();

  // Push button delay time to avoid double command
  if (HAL_GetTick() == BUTTON_DELAY && Enable_start_button != TRUE)
    Enable_start_button = TRUE;

  // SIXSTEP_parameters.Speed_Loop_Time x 1msec
  if (Tick_cnt >= SIXSTEP_parameters.Speed_Loop_Time) {
    if (SIXSTEP_parameters.STATUS != SPEEDFBKERROR)
      MC_Task_Speed();

    SIXSTEP_parameters.MediumFrequencyTask_flag = TRUE;
    if (SIXSTEP_parameters.VALIDATION_OK == TRUE)
      MC_Set_Speed(0);
    Tick_cnt = 0;
    }
  else
    Tick_cnt++;

  if (startup_bemf_failure == 1) {
    SIXSTEP_parameters.ACCEL >>= 1;
    if (SIXSTEP_parameters.ACCEL < MINIMUM_ACC)
      SIXSTEP_parameters.ACCEL = MINIMUM_ACC;
    MC_StopMotor();
    cnt_bemf_event = 0;
    SIXSTEP_parameters.STATUS = STARTUP_BEMF_FAILURE;
    }

  if (speed_fdbk_error == 1) {
    MC_StopMotor();
    SIXSTEP_parameters.STATUS = SPEEDFBKERROR;
    }
  }
//}}}

// callback actions
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc) {
  MC_ADCx_SixStep_Bemf();
  }

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef* htim) {
  MC_TIMx_SixStep_timebase();
  }

void HAL_SYSTICK_Callback() {
  MC_SysTick_SixStep_MediumFrequencyTask();
  }

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {
  MC_EXT_button_SixStep();
  }

// external interface
//{{{
void MC_SixStep_INIT() {

  MC_SixStep_Nucleo_Init();

  SIXSTEP_parameters.HF_TIMx_CCR = HF_TIMx.Instance->HF_TIMx_CCR1;
  SIXSTEP_parameters.HF_TIMx_ARR = HF_TIMx.Instance->ARR;
  SIXSTEP_parameters.HF_TIMx_PSC = HF_TIMx.Instance->PSC;
  SIXSTEP_parameters.LF_TIMx_ARR = LF_TIMx.Instance->ARR;
  SIXSTEP_parameters.LF_TIMx_PSC = LF_TIMx.Instance->PSC;

  // MC_SixStep_Current_Reference_Start();
  MC_SixStep_Current_Reference_Setvalue (SIXSTEP_parameters.Ireference);

  #ifdef UART_COMM
    SIXSTEP_parameters.Button_ready = FALSE;
    MC_UI_INIT();
  #endif

  SIXSTEP_parameters.Ireference = STARTUP_CURRENT_REFERENCE;
  SIXSTEP_parameters.NUMPOLESPAIRS = NUM_POLE_PAIRS;
  SIXSTEP_parameters.ACCEL = ACC;
  SIXSTEP_parameters.KP = KP_GAIN;
  SIXSTEP_parameters.KI = KI_GAIN;
  SIXSTEP_parameters.CW_CCW = DIRECTION;
  SIXSTEP_parameters.Potentiometer = POTENTIOMETER;

  #ifndef UART_COMM
    SIXSTEP_parameters.Button_ready = TRUE;
  #endif

  MC_SixStep_RESET();
  }
//}}}
//{{{
void MC_SixStep_RESET() {

  SIXSTEP_parameters.CMD = TRUE;
  SIXSTEP_parameters.numberofitemArr = NUMBER_OF_STEPS;
  SIXSTEP_parameters.ADC_BEMF_threshold_UP = BEMF_THRSLD_UP;
  SIXSTEP_parameters.ADC_BEMF_threshold_DOWN = BEMF_THRSLD_DOWN;
  SIXSTEP_parameters.Ireference = STARTUP_CURRENT_REFERENCE;
  SIXSTEP_parameters.Speed_Loop_Time = SPEED_LOOP_TIME;
  SIXSTEP_parameters.pulse_value = SIXSTEP_parameters.HF_TIMx_CCR;
  SIXSTEP_parameters.Speed_target_ramp = MAX_POT_SPEED;
  SIXSTEP_parameters.ALIGNMENT = FALSE;
  SIXSTEP_parameters.Speed_Ref_filtered = 0;
  SIXSTEP_parameters.demagn_value = INITIAL_DEMAGN_DELAY;

  SIXSTEP_parameters.CurrentRegular_BEMF_ch = 0;
  SIXSTEP_parameters.status_prev = 0;
  SIXSTEP_parameters.step_position = 0;

  LF_TIMx.Init.Prescaler = SIXSTEP_parameters.LF_TIMx_PSC;
  LF_TIMx.Instance->PSC =  SIXSTEP_parameters.LF_TIMx_PSC;
  LF_TIMx.Init.Period =    SIXSTEP_parameters.LF_TIMx_ARR;
  LF_TIMx.Instance->ARR =  SIXSTEP_parameters.LF_TIMx_ARR;
  HF_TIMx.Init.Prescaler = SIXSTEP_parameters.HF_TIMx_PSC;
  HF_TIMx.Instance->PSC =  SIXSTEP_parameters.HF_TIMx_PSC;
  HF_TIMx.Init.Period =    SIXSTEP_parameters.HF_TIMx_ARR;
  HF_TIMx.Instance->ARR =  SIXSTEP_parameters.HF_TIMx_ARR;
  HF_TIMx.Instance->HF_TIMx_CCR1 = SIXSTEP_parameters.HF_TIMx_CCR;

  Rotor_poles_pairs = SIXSTEP_parameters.NUMPOLESPAIRS;
  SIXSTEP_parameters.SYSCLK_frequency = HAL_RCC_GetSysClockFreq();

  MC_SixStep_HF_TIMx_SetDutyCycle_CH1(0);
  MC_SixStep_HF_TIMx_SetDutyCycle_CH2(0);
  MC_SixStep_HF_TIMx_SetDutyCycle_CH3(0);

  SIXSTEP_parameters.Regular_channel[1] = ADC_Bemf_CH1;   /*BEMF1*/
  SIXSTEP_parameters.Regular_channel[2] = ADC_Bemf_CH2;   /*BEMF2*/
  SIXSTEP_parameters.Regular_channel[3] = ADC_Bemf_CH3;   /*BEMF3*/
  SIXSTEP_parameters.ADC_SEQ_CHANNEL[0] = ADC_CH_1;       /*CURRENT*/
  SIXSTEP_parameters.ADC_SEQ_CHANNEL[1] = ADC_CH_2;       /*SPEED*/
  SIXSTEP_parameters.ADC_SEQ_CHANNEL[2] = ADC_CH_3;       /*VBUS*/
  SIXSTEP_parameters.ADC_SEQ_CHANNEL[3] = ADC_CH_4;       /*TEMP*/

  SIXSTEP_parameters.step_position = 0;
  SIXSTEP_parameters.demagn_counter = 0;
  SIXSTEP_parameters.ALIGN_OK = FALSE;
  SIXSTEP_parameters.VALIDATION_OK = 0;
  SIXSTEP_parameters.ARR_OK = 0;
  SIXSTEP_parameters.speed_fdbk_filtered = 0;
  SIXSTEP_parameters.Integral_Term_sum = 0;
  SIXSTEP_parameters.Current_Reference = 0;
  SIXSTEP_parameters.Ramp_Start = 0;
  SIXSTEP_parameters.RUN_Motor = 0;
  SIXSTEP_parameters.speed_fdbk = 0;
  SIXSTEP_parameters.BEMF_OK = FALSE;
  SIXSTEP_parameters.CL_READY = FALSE;
  SIXSTEP_parameters.SPEED_VALIDATED = FALSE;
  SIXSTEP_parameters.BEMF_Tdown_count = 0;   /* Reset of the Counter to detect Stop motor condition when a stall condition occurs*/

  index_motor_run = 0;
  test_motor_run = 1;
  T_single_step = 0;
  T_single_step_first_value = 0;
  delta = 0;
  Time_vector_tmp = 0;
  Time_vector_prev_tmp = 0;
  Mech_Speed_RPM = 0;
  El_Speed_Hz = 0;
  index_adc_chn = 0;
  mech_accel_hz = 0;
  constant_k = 0;
  ARR_LF = 0;
  index_array = 1;
  Enable_start_button = TRUE;
  index_ARR_step = 1;
  n_zcr_startup = 0;
  cnt_bemf_event = 0;
  startup_bemf_failure = 0;
  speed_fdbk_error = 0;

  index_align = 1;
  speed_sum_sp_filt = 0;
  speed_sum_pot_filt = 0;
  index_pot_filt = 1;
  potent_filtered = 0;
  Tick_cnt = 0;
  counter_ARR_Bemf = 0;
  constant_multiplier_tmp = 0;

  HFBufferIndex =0;
  for (uint16_t i = 0; i < HFBUFFERSIZE;i++)
    HFBuffer[i]=0;

  for (uint16_t i = 0; i < FILTER_DEEP;i++) {
    speed_tmp_array[i] = 0;
    speed_tmp_buffer[i]= 0;
    }
  array_completed = FALSE;
  buffer_completed = FALSE;

  if (PI_parameters.Reference < 0)
    SIXSTEP_parameters.step_position = 1;

  target_speed = TARGET_SPEED;
  MC_Set_PI_param (&PI_parameters);
  MC_SixStep_Current_Reference_Start();
  MC_SixStep_Current_Reference_Setvalue (SIXSTEP_parameters.Ireference);

  index_startup_motor = 1;
  MC_SixStep_Ramp_Motor_calc();
  }
//}}}

//{{{
void MC_StartMotor() {

  SIXSTEP_parameters.STATUS = START;

  HAL_TIM_Base_Start_IT (&LF_TIMx);
  HAL_ADC_Start_IT (&ADCx);

  SIXSTEP_parameters.RUN_Motor = 1;

  BSP_X_NUCLEO_FAULT_LED_ON();

  if (dac_status == TRUE)
    START_DAC();
  }
//}}}
//{{{
void MC_StopMotor() {

  SIXSTEP_parameters.STATUS = STOP;
  SIXSTEP_parameters.RUN_Motor = 0;
  MC_SixStep_Stop_PWM_driving();

  HF_TIMx.Instance->CR1 &= ~(TIM_CR1_CEN);
  HF_TIMx.Instance->CNT = 0;
  MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D();
  HAL_TIM_Base_Stop_IT (&LF_TIMx);
  HAL_ADC_Stop_IT (&ADCx);

  MC_SixStep_Current_Reference_Stop();
  BSP_X_NUCLEO_FAULT_LED_OFF();

  MC_SixStep_RESET();
  }
//}}}

//{{{
void MC_Set_Speed (uint16_t speed_value) {

#if (POTENTIOMETER == 1)
  uint8_t change_target_speed = 0;
  int16_t reference_tmp = 0;

  if (SIXSTEP_parameters.Speed_Ref_filtered > SIXSTEP_parameters.Speed_target_ramp) {
    if ((SIXSTEP_parameters.Speed_Ref_filtered - SIXSTEP_parameters.Speed_target_ramp) > ADC_SPEED_TH)
      change_target_speed = 1;
    else {
      /* Not change target speed because less than threshold */
      }
    }
  else {
    if ((SIXSTEP_parameters.Speed_target_ramp - SIXSTEP_parameters.Speed_Ref_filtered) > ADC_SPEED_TH)
      change_target_speed = 1;
    else {
      /* Not change target speed because less than threshold */
      }
    }

  if (change_target_speed == 1) {
    SIXSTEP_parameters.Speed_target_ramp = SIXSTEP_parameters.Speed_Ref_filtered;

    if (SIXSTEP_parameters.CW_CCW == 0) {
      reference_tmp = SIXSTEP_parameters.Speed_Ref_filtered * MAX_POT_SPEED / 4096;
       if(reference_tmp <= MIN_POT_SPEED)
         PI_parameters.Reference = MIN_POT_SPEED;
       else
         PI_parameters.Reference =  reference_tmp;
      }
    else {
      reference_tmp = -(SIXSTEP_parameters.Speed_Ref_filtered * MAX_POT_SPEED / 4096);
       if(reference_tmp >=- MIN_POT_SPEED)
         PI_parameters.Reference = -MIN_POT_SPEED;
       else
         PI_parameters.Reference=  reference_tmp;
      }
    }
#else
  if (speed_value != 0)
    PI_parameters.Reference = speed_value;
#endif
  }
//}}}
//{{{
void MC_EXT_button_SixStep() {

  if (Enable_start_button == TRUE) {
    if ((SIXSTEP_parameters.RUN_Motor == 0) && (SIXSTEP_parameters.Button_ready == TRUE)) {
      printf ("StartMotor\n");
      MC_StartMotor();
      Enable_start_button = FALSE;
      }
    else {
      printf ("StopMotor\n");
      MC_StopMotor();
      Enable_start_button = FALSE;
      }
    }
  }
//}}}
