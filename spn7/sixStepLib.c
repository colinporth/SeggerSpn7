// 6StepLib.c
#include "sixStepLib.h"

SIXSTEP_Base_InitTypeDef sixStep;  // Main SixStep structure
SIXSTEP_PI_PARAM_InitTypeDef_t PI_parameters; // SixStep PI regulator structure
//{{{  vars
uint16_t Rotor_poles_pairs;               //  Number of pole pairs of the motor
uint32_t mech_accel_hz = 0;               //  Hz -- Mechanical acceleration rate
uint32_t constant_k = 0;                  //  1/3*mech_accel_hz

uint32_t Time_vector_tmp = 0;             //  Startup variable
uint32_t Time_vector_prev_tmp = 0 ;       //  Startup variable
uint32_t T_single_step = 0;               //  Startup variable
uint32_t T_single_step_first_value = 0;   //  Startup variable

int32_t  delta = 0;                       //  Startup variable
uint16_t index_array = 1;                 //  Speed filter variable
int16_t speed_tmp_array[FILTER_DEEP];     //  Speed filter variable
uint16_t speed_tmp_buffer[FILTER_DEEP];   //  Potentiometer filter variable
uint16_t HFBuffer[HFBUFFERSIZE];          //  Buffer for Potentiometer Value Filtering at the High-Frequency ADC conversion
uint16_t HFBufferIndex = 0;               //  High-Frequency Buffer Index
uint8_t  array_completed = FALSE;         //  Speed filter variable
uint8_t  buffer_completed = FALSE;        //  Potentiometer filter variable

uint32_t ARR_LF = 0;                      //  Autoreload LF TIM variable
int32_t Mech_Speed_RPM = 0;               //  Mechanical motor speed
int32_t El_Speed_Hz = 0;                  //  Electrical motor speed

uint16_t index_adc_chn = 0;               //  Index of ADC channel selector for measuring
uint16_t index_motor_run = 0;             //  Tmp variable for DEMO mode

uint16_t test_motor_run = 1;              //  Tmp variable for DEMO mode
uint8_t Enable_start_button = TRUE;       //  Start/stop button filter to avoid double command

uint16_t index_ARR_step = 1;
uint32_t n_zcr_startup = 0;
uint16_t index_startup_motor = 1;
uint16_t target_speed = TARGET_SPEED;     //  Target speed for closed loop control
uint16_t shift_n_sqrt = 14;

uint16_t cnt_bemf_event = 0;
uint8_t startup_bemf_failure = 0;

uint8_t speed_fdbk_error = 0;

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
static uint64_t fastSqrt (uint64_t wInput) {

  uint64_t tempRoot;
  if (wInput <= (uint64_t)((uint64_t)2097152 << shift_n_sqrt))
    tempRoot = (uint64_t)((uint64_t)128 << shift_n_sqrt);
  else
    tempRoot = (uint64_t)((uint64_t)8192 << shift_n_sqrt);

  uint8_t biter = 0u;
  uint64_t tempRootNew;
  do {
    tempRootNew = (tempRoot + wInput / tempRoot) >> 1;
    if (tempRootNew == tempRoot)
      biter = shift_n_sqrt -1 ;
    else {
      biter ++;
      tempRoot = tempRootNew;
      }
    } while (biter < (shift_n_sqrt - 1));

  return tempRootNew;
  }
//}}}

//{{{
static void potSpeedTarget() {

  target_speed = sixStep.ADC_Regular_Buffer[1] * MAX_POT_SPEED / 4096;

  if (target_speed < MIN_POT_SPEED)
    target_speed = MIN_POT_SPEED;
  if (target_speed > (MAX_POT_SPEED / VAL_POT_SPEED_DIV))
    target_speed = (MAX_POT_SPEED/VAL_POT_SPEED_DIV);
  }
//}}}
//{{{
static void potSpeed() {

  uint32_t sum = 0;
  uint16_t max = 0;
  for (uint16_t i = 0; i < HFBUFFERSIZE; i++) {
    uint16_t val = HFBuffer[i];
    sum += val;
    if (val > max)
      max = val;
    }

  sum -= max;
  uint16_t potMean = sum / (HFBUFFERSIZE - 1);

  if (buffer_completed == FALSE) {
    speed_tmp_buffer[index_pot_filt] = potMean;
    speed_sum_pot_filt = 0;
    for (uint16_t i = 1; i <= index_pot_filt;i++)
      speed_sum_pot_filt = speed_sum_pot_filt + speed_tmp_buffer[i];
    potent_filtered = speed_sum_pot_filt / index_pot_filt;
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
     speed_tmp_buffer[index_pot_filt] = potMean;
     uint16_t speed_max = 0;
     for (uint16_t i = 1; i < FILTER_DEEP;i++) {
       uint16_t val = speed_tmp_buffer[i];
       if (val > speed_max)
         speed_max = val;
       speed_sum_pot_filt += val;
       }
     speed_sum_pot_filt -= speed_max;
     potent_filtered = speed_sum_pot_filt / (FILTER_DEEP-2);
    }

  if (potent_filtered==0)
    potent_filtered = 1;

  sixStep.Speed_Ref_filtered = potent_filtered;
  }
//}}}
//{{{
static void speedFilter() {

  if (array_completed == FALSE) {
    speed_tmp_array[index_array] = sixStep.speed_fdbk;
    speed_sum_sp_filt = 0;
    for (uint16_t i = 1; i <= index_array;i++)
      speed_sum_sp_filt = speed_sum_sp_filt + speed_tmp_array[i];
    sixStep.speed_fdbk_filtered = speed_sum_sp_filt / index_array;
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
    speed_tmp_array[index_array] = sixStep.speed_fdbk;
    for (uint16_t i = 1; i < FILTER_DEEP;i++)
      speed_sum_sp_filt = speed_sum_sp_filt + speed_tmp_array[i];
    sixStep.speed_fdbk_filtered = speed_sum_sp_filt / (FILTER_DEEP-1);
    }
  }
//}}}

//{{{
static void sixStepTable (uint8_t step_number) {

  switch (step_number) {
    case 1:
      MC_HF_TIMx_SetDutyCycle_CH1 (sixStep.pulse_value);
      MC_HF_TIMx_SetDutyCycle_CH2 (0);
      MC_HF_TIMx_SetDutyCycle_CH3 (0);
      MC_EnableInput_CH1_E_CH2_E_CH3_D();
      sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[3];
      break;

    case 2:
      MC_HF_TIMx_SetDutyCycle_CH1 (sixStep.pulse_value);
      MC_HF_TIMx_SetDutyCycle_CH2 (0);
      MC_HF_TIMx_SetDutyCycle_CH3 (0);
      MC_EnableInput_CH1_E_CH2_D_CH3_E();
      sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[2];
      break;

    case 3:
      MC_HF_TIMx_SetDutyCycle_CH2 (sixStep.pulse_value);
      MC_HF_TIMx_SetDutyCycle_CH3 (0);
      MC_HF_TIMx_SetDutyCycle_CH1 (0);
      MC_EnableInput_CH1_D_CH2_E_CH3_E();
      sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[1];
      break;

    case 4:
      MC_HF_TIMx_SetDutyCycle_CH2 (sixStep.pulse_value);
      MC_HF_TIMx_SetDutyCycle_CH1 (0);
      MC_HF_TIMx_SetDutyCycle_CH3 (0);
      MC_EnableInput_CH1_E_CH2_E_CH3_D();
      sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[3];
     break;

    case 5:
      MC_HF_TIMx_SetDutyCycle_CH3 (sixStep.pulse_value);
      MC_HF_TIMx_SetDutyCycle_CH1 (0);
      MC_HF_TIMx_SetDutyCycle_CH2 (0);
      MC_EnableInput_CH1_E_CH2_D_CH3_E();
      sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[2];
      break;

    case 6:
      MC_HF_TIMx_SetDutyCycle_CH3 (sixStep.pulse_value);
      MC_HF_TIMx_SetDutyCycle_CH2 (0);
      MC_HF_TIMx_SetDutyCycle_CH1 (0);
      MC_EnableInput_CH1_D_CH2_E_CH3_E();
      sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[1];
      break;
     }
  }
//}}}
//{{{
static void rampMotorCalc() {

  uint32_t constant_multiplier = 100;
  uint32_t constant_multiplier_2 = 4000000000;

  if (index_startup_motor == 1) {
    mech_accel_hz = sixStep.ACCEL * Rotor_poles_pairs / 60;
    constant_multiplier_tmp = (uint64_t)constant_multiplier * (uint64_t)constant_multiplier_2;
    constant_k = constant_multiplier_tmp / (3 * mech_accel_hz);
    MC_Current_Reference_Setvalue (sixStep.Ireference);
    Time_vector_prev_tmp = 0;
    }

  if (index_startup_motor < NUMBER_OF_STEPS) {
    Time_vector_tmp = ((uint64_t)1000 * (uint64_t)1000 * (uint64_t) fastSqrt(((uint64_t)index_startup_motor * (uint64_t)constant_k)))/632455;
    delta = Time_vector_tmp - Time_vector_prev_tmp;
    if (index_startup_motor == 1) {
      T_single_step_first_value = (2 * 3141) * delta / 1000;
      sixStep.ARR_value = (uint32_t)(65535);
      }
    else {
      T_single_step = (2 * 3141)* delta / 1000;
      sixStep.ARR_value = (uint32_t)(65535 * T_single_step) / T_single_step_first_value;
      }
    }
  else
    index_startup_motor = 1;

  if (index_startup_motor == 1)
    sixStep.prescaler_value = (((sixStep.SYSCLK_frequency / 1000000) * T_single_step_first_value)/65535) - 1;

  if ((sixStep.STATUS != ALIGNMENT) && (sixStep.STATUS != START))
    index_startup_motor++;
  else
    Time_vector_tmp = 0;

  Time_vector_prev_tmp = Time_vector_tmp;
  }
//}}}

//{{{
static void nextStep() {

  if (sixStep.CMD == TRUE) {
    sixStep.CMD = FALSE;
    MC_Start_PWM();
    }
  ARR_LF = __HAL_TIM_GetAutoreload (&LF_TIMx);

  if (sixStep.ALIGN_OK == TRUE) {
    sixStep.speed_fdbk = MC_GetMechSpeedRPM();
    sixStep.demagn_counter = 1;
    if (sixStep.status_prev != sixStep.step_position)
      n_zcr_startup = 0;

    if (PI_parameters.Reference>=0) {
      sixStep.step_position++;
      if (sixStep.CL_READY == TRUE)
        sixStep.VALIDATION_OK = TRUE;
      if (sixStep.step_position > 6)
        sixStep.step_position = 1;
      }
    else {
      sixStep.step_position--;
      if (sixStep.CL_READY == TRUE)
        sixStep.VALIDATION_OK = TRUE;
      if (sixStep.step_position < 1)
        sixStep.step_position = 6;
      }
    }

  if (sixStep.VALIDATION_OK == 1) {
    // Motor Stall condition detection and Speed-Feedback error generation
    sixStep.BEMF_Tdown_count++;
    if (sixStep.BEMF_Tdown_count > BEMF_CONSEC_DOWN_MAX)
      speed_fdbk_error = 1;
    else
      __HAL_TIM_SetAutoreload (&LF_TIMx, 0xFFFF);
    }
  sixStepTable (sixStep.step_position);

  // It controls if the changing step request appears during DOWNcounting
  // in this case it changes the ADC channel UP-COUNTING direction started DIR = 0
  if (__HAL_TIM_DIRECTION_STATUS (&HF_TIMx)) {
    switch (sixStep.step_position) {
      case 1: sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[3]; break;
      case 2: sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[2]; break;
      case 3: sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[1]; break;
      case 4: sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[3]; break;
      case 5: sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[2]; break;
      case 6: sixStep.CurrentRegular_BEMF_ch = sixStep.Regular_channel[1]; break;
      }

    MC_ADC_Channel (sixStep.CurrentRegular_BEMF_ch);
    }
  }
//}}}
//{{{
static void arrStep() {

  if (sixStep.ALIGNMENT == FALSE)
    sixStep.ALIGNMENT = TRUE;

  if (sixStep.ALIGN_OK == TRUE) {
    if (PI_parameters.Reference >= 0) {
      if (sixStep.VALIDATION_OK != TRUE) {
        sixStep.STATUS = STARTUP;
        rampMotorCalc();
        if (index_ARR_step < sixStep.numberofitemArr) {
          LF_TIMx.Init.Period = sixStep.ARR_value;
          LF_TIMx.Instance->ARR = (uint32_t)LF_TIMx.Init.Period;
          index_ARR_step++;
          }
        else if (sixStep.ARR_OK == 0) {
          index_ARR_step = 1;
          sixStep.ACCEL >>= 1;
          if (sixStep.ACCEL < MINIMUM_ACC)
            sixStep.ACCEL = MINIMUM_ACC;
          MC_StopMotor();
          sixStep.STATUS = STARTUP_FAILURE;
          }
        }
      else {
        sixStep.ARR_OK = 1;
        index_startup_motor = 1;
        index_ARR_step = 1;
        }
      }
    else {
      if (sixStep.VALIDATION_OK != TRUE) {
        sixStep.STATUS = STARTUP;
        rampMotorCalc();
        if (index_ARR_step < sixStep.numberofitemArr) {
          LF_TIMx.Init.Period = sixStep.ARR_value;
          LF_TIMx.Instance->ARR = (uint32_t)LF_TIMx.Init.Period;
          index_ARR_step++;
          }
        else if(sixStep.ARR_OK == 0) {
          index_ARR_step = 1;
          sixStep.ACCEL >>= 1;
          if (sixStep.ACCEL < MINIMUM_ACC)
            sixStep.ACCEL = MINIMUM_ACC;
          MC_StopMotor();
          sixStep.STATUS = STARTUP_FAILURE;
          }
        }
      else {
        sixStep.ARR_OK = 1;
        index_startup_motor = 1;
        index_ARR_step = 1;
        }
      }
    }
  }
//}}}
//{{{
void arrBemf (uint8_t up_bemf) {

  if (sixStep.status_prev != sixStep.step_position) {
    if (sixStep.SPEED_VALIDATED == TRUE) {
       if (cnt_bemf_event> BEMF_CNT_EVENT_MAX)
        startup_bemf_failure = 1;

      if ((up_bemf == 1) && (sixStep.BEMF_OK != TRUE)) {
        n_zcr_startup++;
        cnt_bemf_event = 0;
        }
      else if (sixStep.BEMF_OK != TRUE)
        cnt_bemf_event++;

      if ((n_zcr_startup >= NUMBER_ZCR) && (sixStep.BEMF_OK != TRUE)) {
        sixStep.BEMF_OK = TRUE;
        n_zcr_startup = 0;
        }
      }

    sixStep.status_prev = sixStep.step_position;

    if (sixStep.VALIDATION_OK == 1) {
      counter_ARR_Bemf = __HAL_TIM_GetCounter (&LF_TIMx);
      __HAL_TIM_SetAutoreload (&LF_TIMx, counter_ARR_Bemf + ARR_LF / 2);
      }
    }
  }
//}}}

//{{{
static void setPiParam (SIXSTEP_PI_PARAM_InitTypeDef_t *PI_PARAM) {

  if (sixStep.CW_CCW == 0)
    PI_PARAM->Reference = target_speed;
  else
    PI_PARAM->Reference = -target_speed;

  PI_PARAM->Kp_Gain = sixStep.KP;
  PI_PARAM->Ki_Gain = sixStep.KI;

  PI_PARAM->Lower_Limit_Output = LOWER_OUT_LIMIT;
  PI_PARAM->Upper_Limit_Output = UPPER_OUT_LIMIT;
  PI_PARAM->Max_PID_Output = FALSE;
  PI_PARAM->Min_PID_Output = FALSE;
  }
//}}}
//{{{
static int16_t piController (SIXSTEP_PI_PARAM_InitTypeDef_t *PI_PARAM, int16_t speed_fdb) {

  int32_t Error = (PI_PARAM->Reference - speed_fdb);

  // Proportional term computation
  int32_t wProportional_Term = PI_PARAM->Kp_Gain * Error;

  // Integral term computation
  int32_t wIntegral_Term = 0;
  int32_t wIntegral_sum_temp = 0;
  if (PI_PARAM->Ki_Gain == 0)
    sixStep.Integral_Term_sum = 0;
  else {
    wIntegral_Term = PI_PARAM->Ki_Gain * Error;
    wIntegral_sum_temp = sixStep.Integral_Term_sum + wIntegral_Term;
    sixStep.Integral_Term_sum = wIntegral_sum_temp;
    }

  if (sixStep.Integral_Term_sum > KI_DIV * PI_PARAM->Upper_Limit_Output)
    sixStep.Integral_Term_sum = KI_DIV* PI_PARAM->Upper_Limit_Output;

  if (sixStep.Integral_Term_sum < -KI_DIV* PI_PARAM->Upper_Limit_Output)
    sixStep.Integral_Term_sum = -KI_DIV* PI_PARAM->Upper_Limit_Output;

  // WARNING: the below instruction is not MISRA compliant, user should verify
  //          that Cortex-M3 assembly instruction ASR (arithmetic shift right)
  //          is used by the compiler to perform the shifts (instead of LSR logical shift right)
  int32_t wOutput_32 = (wProportional_Term / KP_DIV) + (sixStep.Integral_Term_sum / KI_DIV);

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
static void taskSpeed() {

  if ((sixStep.speed_fdbk_filtered > (target_speed) ||
       sixStep.speed_fdbk_filtered < (-target_speed)) &&
       sixStep.VALIDATION_OK != TRUE) {
    sixStep.STATUS = VALIDATION;
    sixStep.SPEED_VALIDATED = TRUE;
    }

  if (sixStep.SPEED_VALIDATED == TRUE &&
      sixStep.BEMF_OK == TRUE &&
      sixStep.CL_READY != TRUE)
    sixStep.CL_READY = TRUE;

  if (sixStep.VALIDATION_OK == TRUE) {
    sixStep.STATUS = RUN;
    if (PI_parameters.Reference >= 0)
      sixStep.Current_Reference = (uint16_t)piController (&PI_parameters,(int16_t)sixStep.speed_fdbk_filtered);
    else
      sixStep.Current_Reference = (uint16_t)(-piController (&PI_parameters,(int16_t)sixStep.speed_fdbk_filtered));

    MC_Current_Reference_Setvalue (sixStep.Current_Reference);
    }

  bemfDelayCalc();
  }
//}}}

// callback interface
//{{{
void MC_ADC() {

  if (__HAL_TIM_DIRECTION_STATUS (&HF_TIMx)) {
    // UP-counting direction started, GET the ADC value (PHASE CURRENT)
    if ((sixStep.STATUS != START) && (sixStep.STATUS != ALIGNMENT)) {
      switch (sixStep.step_position) {
        //{{{
        case 1:
          if (sixStep.demagn_counter >= sixStep.demagn_value) {
            sixStep.ADC_BUFFER[3] = HAL_ADC_GetValue (&ADCx);
            if (PI_parameters.Reference >= 0) {
              if (sixStep.ADC_BUFFER[3] < sixStep.ADC_BEMF_threshold_DOWN)
                arrBemf (0);
              }
            else if (sixStep.ADC_BUFFER[3] > sixStep.ADC_BEMF_threshold_UP) {
              arrBemf (1);
              sixStep.BEMF_Tdown_count = 0;
              }
            }
          else
            sixStep.demagn_counter++;

          break;
        //}}}
        //{{{
        case 2:
          if (sixStep.demagn_counter >= sixStep.demagn_value) {
            sixStep.ADC_BUFFER[2] = HAL_ADC_GetValue (&ADCx);
            if (PI_parameters.Reference >= 0) {
              if (sixStep.ADC_BUFFER[2] > sixStep.ADC_BEMF_threshold_UP) {
                arrBemf (1);
                sixStep.BEMF_Tdown_count = 0;
                }
              }
            else if (sixStep.ADC_BUFFER[2] < sixStep.ADC_BEMF_threshold_DOWN)
              arrBemf (0);
            }
          else
           sixStep.demagn_counter++;

          break;
        //}}}
        //{{{
        case 3:
          if (sixStep.demagn_counter >= sixStep.demagn_value) {
            sixStep.ADC_BUFFER[1] = HAL_ADC_GetValue (&ADCx);
            if (PI_parameters.Reference >= 0) {
              if (sixStep.ADC_BUFFER[1] < sixStep.ADC_BEMF_threshold_DOWN)
                arrBemf (0);
              }
            else if (sixStep.ADC_BUFFER[1] > sixStep.ADC_BEMF_threshold_UP) {
              arrBemf (1);
              sixStep.BEMF_Tdown_count = 0;
              }
            }
          else
            sixStep.demagn_counter++;

          break;
        //}}}
        //{{{
        case 4:
         if (sixStep.demagn_counter >= sixStep.demagn_value) {
           sixStep.ADC_BUFFER[3] = HAL_ADC_GetValue (&ADCx);
           if (PI_parameters.Reference >= 0) {
             if (sixStep.ADC_BUFFER[3] > sixStep.ADC_BEMF_threshold_UP) {
               arrBemf (1);
               sixStep.BEMF_Tdown_count = 0;
               }
             }
           else if (sixStep.ADC_BUFFER[3] < sixStep.ADC_BEMF_threshold_DOWN)
             arrBemf (0);
           }
         else
           sixStep.demagn_counter++;

         break;
        //}}}
        //{{{
        case 5:
          if (sixStep.demagn_counter >= sixStep.demagn_value) {
           sixStep.ADC_BUFFER[2] = HAL_ADC_GetValue (&ADCx);
           if (PI_parameters.Reference >= 0) {
             if (sixStep.ADC_BUFFER[2] < sixStep.ADC_BEMF_threshold_DOWN)
               arrBemf (0);
             }
           else if (sixStep.ADC_BUFFER[2] > sixStep.ADC_BEMF_threshold_UP) {
             arrBemf (1);
             sixStep.BEMF_Tdown_count = 0;
             }
           }
         else
           sixStep.demagn_counter++;

         break;
        //}}}
        //{{{
        case 6:
          if (sixStep.demagn_counter >= sixStep.demagn_value) {
            sixStep.ADC_BUFFER[1] = HAL_ADC_GetValue(&ADCx);
            if (PI_parameters.Reference>=0) {
             if (sixStep.ADC_BUFFER[1] > sixStep.ADC_BEMF_threshold_UP) {
               arrBemf (1);
               sixStep.BEMF_Tdown_count = 0;
               }
             }
           else if (sixStep.ADC_BUFFER[1] < sixStep.ADC_BEMF_threshold_DOWN)
             arrBemf (0);
           }
         else
           sixStep.demagn_counter++;
          break;
        //}}}
        }
      //printf ("%d adcU %d\n", HAL_GetTick(), sixStep.step_position);
      }

    // SET ADC CHANNEL FOR SPEED/CURRENT/VBUS
    // Set the channel for next ADC Regular reading
    MC_ADC_Channel (sixStep.ADC_SEQ_CHANNEL[index_adc_chn]);
    }

  else {
    // Down-counting direction started, Set the channel for next ADC Regular reading
    sixStep.ADC_Regular_Buffer[index_adc_chn] = HAL_ADC_GetValue (&ADCx);
    if (index_adc_chn == 1) {
      HFBuffer[HFBufferIndex++] = HAL_ADC_GetValue (&ADCx);
      if (HFBufferIndex >= HFBUFFERSIZE)
        HFBufferIndex = 0;
      }

    //printf ("%d adcD %d\n", HAL_GetTick(), index_adc_chn);

    index_adc_chn++;
    if (index_adc_chn > 3)
      index_adc_chn = 0;

    MC_ADC_Channel (sixStep.CurrentRegular_BEMF_ch);
    }
  }
//}}}
//{{{
void MC_Timebase() {

  nextStep();

  // BASE TIMER - ARR modification for STEP frequency changing
  if (sixStep.ARR_OK == 0)
    arrStep();

  speedFilter();
  }
//}}}
//{{{
void MC_SysTick() {

  if ((sixStep.ALIGNMENT == TRUE) && (sixStep.ALIGN_OK == FALSE)) {
    //{{{  align motor
    sixStep.step_position = 6;
    LF_TIMx.Init.Period = sixStep.ARR_value;
    LF_TIMx.Instance->ARR = (uint32_t)LF_TIMx.Init.Period;
    sixStep.STATUS = ALIGNMENT;

    potSpeedTarget();

    index_align++;
    if (index_align >= TIME_FOR_ALIGN + 1) {
      printf ("%d aligned\n", HAL_GetTick());
      sixStep.ALIGN_OK = TRUE;
      sixStep.STATUS = STARTUP;
      index_startup_motor = 1;
      rampMotorCalc();
      LF_TIMx.Init.Prescaler = sixStep.prescaler_value;
      LF_TIMx.Instance->PSC = LF_TIMx.Init.Prescaler;
      index_align = 0;
      }
    }
    //}}}

  if (sixStep.VALIDATION_OK == TRUE)
    potSpeed();

  // Push button delay time to avoid double command
  if (HAL_GetTick() == BUTTON_DELAY && Enable_start_button != TRUE)
    Enable_start_button = TRUE;

  // sixStep.Speed_Loop_Time x 1msec
  if (Tick_cnt >= sixStep.Speed_Loop_Time) {
    if (sixStep.STATUS != SPEEDFBKERROR)
      taskSpeed();

    sixStep.MediumFrequencyTask_flag = TRUE;
    if (sixStep.VALIDATION_OK == TRUE)
      MC_SetSpeed (0);
    Tick_cnt = 0;
    }
  else
    Tick_cnt++;

  if (startup_bemf_failure == 1) {
    sixStep.ACCEL >>= 1;
    if (sixStep.ACCEL < MINIMUM_ACC)
      sixStep.ACCEL = MINIMUM_ACC;
    MC_StopMotor();
    cnt_bemf_event = 0;
    sixStep.STATUS = STARTUP_BEMF_FAILURE;
    }

  if (speed_fdbk_error == 1) {
    MC_StopMotor();
    sixStep.STATUS = SPEEDFBKERROR;
    }
  }
//}}}

// external interface
//{{{
void MC_Init() {

  MC_Nucleo_Init();

  sixStep.HF_TIMx_CCR = HF_TIMx.Instance->HF_TIMx_CCR1;
  sixStep.HF_TIMx_ARR = HF_TIMx.Instance->ARR;
  sixStep.HF_TIMx_PSC = HF_TIMx.Instance->PSC;
  sixStep.LF_TIMx_ARR = LF_TIMx.Instance->ARR;
  sixStep.LF_TIMx_PSC = LF_TIMx.Instance->PSC;

  sixStep.Ireference = STARTUP_CURRENT_REFERENCE;
  sixStep.NUMPOLESPAIRS = NUM_POLE_PAIRS;
  sixStep.ACCEL = ACC;
  sixStep.KP = KP_GAIN;
  sixStep.KI = KI_GAIN;
  sixStep.CW_CCW = DIRECTION;
  sixStep.Button_ready = TRUE;

  MC_Reset();
  }
//}}}
//{{{
void MC_Reset() {

  sixStep.CMD = TRUE;
  sixStep.numberofitemArr = NUMBER_OF_STEPS;
  sixStep.ADC_BEMF_threshold_UP = BEMF_THRSLD_UP;
  sixStep.ADC_BEMF_threshold_DOWN = BEMF_THRSLD_DOWN;
  sixStep.Ireference = STARTUP_CURRENT_REFERENCE;
  sixStep.Speed_Loop_Time = SPEED_LOOP_TIME;
  sixStep.pulse_value = sixStep.HF_TIMx_CCR;
  sixStep.Speed_target_ramp = MAX_POT_SPEED;
  sixStep.ALIGNMENT = FALSE;
  sixStep.Speed_Ref_filtered = 0;
  sixStep.demagn_value = INITIAL_DEMAGN_DELAY;

  sixStep.CurrentRegular_BEMF_ch = 0;
  sixStep.status_prev = 0;
  sixStep.step_position = 0;

  LF_TIMx.Init.Prescaler = sixStep.LF_TIMx_PSC;
  LF_TIMx.Instance->PSC =  sixStep.LF_TIMx_PSC;
  LF_TIMx.Init.Period =    sixStep.LF_TIMx_ARR;
  LF_TIMx.Instance->ARR =  sixStep.LF_TIMx_ARR;
  HF_TIMx.Init.Prescaler = sixStep.HF_TIMx_PSC;
  HF_TIMx.Instance->PSC =  sixStep.HF_TIMx_PSC;
  HF_TIMx.Init.Period =    sixStep.HF_TIMx_ARR;
  HF_TIMx.Instance->ARR =  sixStep.HF_TIMx_ARR;
  HF_TIMx.Instance->HF_TIMx_CCR1 = sixStep.HF_TIMx_CCR;

  Rotor_poles_pairs = sixStep.NUMPOLESPAIRS;
  sixStep.SYSCLK_frequency = HAL_RCC_GetSysClockFreq();

  MC_HF_TIMx_SetDutyCycle_CH1(0);
  MC_HF_TIMx_SetDutyCycle_CH2(0);
  MC_HF_TIMx_SetDutyCycle_CH3(0);

  sixStep.Regular_channel[1] = ADC_Bemf_CH1;   //BEMF1
  sixStep.Regular_channel[2] = ADC_Bemf_CH2;   //BEMF2
  sixStep.Regular_channel[3] = ADC_Bemf_CH3;   //BEMF3
  sixStep.ADC_SEQ_CHANNEL[0] = ADC_CH_1;       //CURRENT
  sixStep.ADC_SEQ_CHANNEL[1] = ADC_CH_2;       //SPEED
  sixStep.ADC_SEQ_CHANNEL[2] = ADC_CH_3;       //VBUS
  sixStep.ADC_SEQ_CHANNEL[3] = ADC_CH_4;       //TEMP

  sixStep.step_position = 0;
  sixStep.demagn_counter = 0;
  sixStep.ALIGN_OK = FALSE;
  sixStep.VALIDATION_OK = 0;
  sixStep.ARR_OK = 0;
  sixStep.speed_fdbk_filtered = 0;
  sixStep.Integral_Term_sum = 0;
  sixStep.Current_Reference = 0;
  sixStep.Ramp_Start = 0;
  sixStep.RUN_Motor = 0;
  sixStep.speed_fdbk = 0;
  sixStep.BEMF_OK = FALSE;
  sixStep.CL_READY = FALSE;
  sixStep.SPEED_VALIDATED = FALSE;
  sixStep.BEMF_Tdown_count = 0;   // Reset of the Counter to detect Stop motor condition when a stall condition occurs

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
    sixStep.step_position = 1;

  target_speed = TARGET_SPEED;
  setPiParam (&PI_parameters);

  MC_Current_Reference_Start();
  MC_Current_Reference_Setvalue (sixStep.Ireference);

  index_startup_motor = 1;
  rampMotorCalc();
  }
//}}}

//{{{
int32_t MC_GetElSpeedHz() {

  if (__HAL_TIM_GetAutoreload (&LF_TIMx) != 0xFFFF) {
    uint16_t prsc = LF_TIMx.Instance->PSC;
    El_Speed_Hz = (int32_t)((sixStep.SYSCLK_frequency) / (prsc)) /
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
int32_t MC_GetMechSpeedRPM() {

  Mech_Speed_RPM = (int32_t)(MC_GetElSpeedHz() *  60 / Rotor_poles_pairs);
  return (Mech_Speed_RPM);
  }
//}}}

//{{{
void MC_StartMotor() {

  sixStep.STATUS = START;
  sixStep.RUN_Motor = 1;

  HAL_TIM_Base_Start_IT (&LF_TIMx);
  HAL_ADC_Start_IT (&ADCx);

  NUCLEO_LED_ON();
  }
//}}}
//{{{
void MC_StopMotor() {

  sixStep.STATUS = STOP;
  sixStep.RUN_Motor = 0;
  MC_Stop_PWM();

  HF_TIMx.Instance->CR1 &= ~(TIM_CR1_CEN);
  HF_TIMx.Instance->CNT = 0;
  MC_DisableInput_CH1_D_CH2_D_CH3_D();
  HAL_TIM_Base_Stop_IT (&LF_TIMx);
  HAL_ADC_Stop_IT (&ADCx);

  MC_Current_Reference_Stop();
  NUCLEO_LED_OFF();
  MC_Reset();
  }
//}}}

//{{{
void MC_SetSpeed (uint16_t speed_value) {

  uint8_t change_target_speed = 0;
  int16_t reference_tmp = 0;

  if (sixStep.Speed_Ref_filtered > sixStep.Speed_target_ramp) {
    if ((sixStep.Speed_Ref_filtered - sixStep.Speed_target_ramp) > ADC_SPEED_TH)
      change_target_speed = 1;
    }
  else if ((sixStep.Speed_target_ramp - sixStep.Speed_Ref_filtered) > ADC_SPEED_TH)
    change_target_speed = 1;

  if (change_target_speed == 1) {
    sixStep.Speed_target_ramp = sixStep.Speed_Ref_filtered;

    if (sixStep.CW_CCW == 0) {
      reference_tmp = sixStep.Speed_Ref_filtered * MAX_POT_SPEED / 4096;
       if (reference_tmp <= MIN_POT_SPEED)
         PI_parameters.Reference = MIN_POT_SPEED;
       else
         PI_parameters.Reference =  reference_tmp;
      }
    else {
      reference_tmp = -(sixStep.Speed_Ref_filtered * MAX_POT_SPEED / 4096);
      if (reference_tmp >=- MIN_POT_SPEED)
        PI_parameters.Reference = -MIN_POT_SPEED;
      else
        PI_parameters.Reference=  reference_tmp;
      }
    }
  }
//}}}
//{{{
void MC_EXTbutton() {

  if (Enable_start_button == TRUE) {
    if ((sixStep.RUN_Motor == 0) && (sixStep.Button_ready == TRUE)) {
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

// callbacks
//{{{
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc) {
  MC_ADC();
  }
//}}}
//{{{
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef* htim) {
  MC_Timebase();
  }
//}}}
//{{{
void HAL_SYSTICK_Callback() {
  MC_SysTick();
  }
//}}}

//{{{
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {
  MC_EXTbutton();
  }
//}}}
