// main.c
// PC1 -> ADC12_IN7  curr_fdbk2 - 1shunt
// PA1 -> ADC1_IN2   vbus
// PC2 -> ADC12_IN8  temp
// PB1 -> ADC3_IN1   pot
//
// PC3 -> ADC12_IN9  bemf1
// PB0 -> ADC3_IN12  bemf2
// PA7 -> ADC2_IN4   bemf3
//{{{  includes
#include "stm32f3xx_nucleo.h"

#include "ihm07m1.h"
#include "sixStepLib.h"
#include "cTrace.h"
#include "cLcd.h"
//}}}


//{{{  vars
uint32_t mLastButtonPress = 0;
uint16_t mAlignTicks = 1;
uint16_t mMotorStartupCount = 1;

uint32_t mech_accel_hz = 0;               // Hz -- Mechanical acceleration rate
uint32_t constant_k = 0;                  // 1/3*mech_accel_hz

uint32_t Time_vector_tmp = 0;             // Startup variable
uint32_t Time_vector_prev_tmp = 0 ;       // Startup variable
uint32_t T_single_step = 0;               // Startup variable
uint32_t T_single_step_first_value = 0;   // Startup variable

int32_t delta = 0;                        // Startup variable
uint16_t index_array = 1;                 // Speed filter variable
int16_t speed_tmp_array[FILTER_DEEP];     // Speed filter variable
uint16_t speed_tmp_buffer[FILTER_DEEP];   // Potentiometer filter variable
bool array_completed = false;             // Speed filter variable
bool buffer_completed = false;            // Potentiometer filter variable

#define POT_VALUES_SIZE                   10
uint16_t mPotValueIndex = 0;              // High-Frequency Buffer Index
uint16_t mPotValues[POT_VALUES_SIZE];     // Buffer for Potentiometer Value Filtering at the High-Frequency ADC conversion

uint32_t ARR_LF = 0;                      // Autoreload LF TIM variable
int32_t Mech_Speed_RPM = 0;               // Mechanical motor speed
int32_t El_Speed_Hz = 0;                  // Electrical motor speed

uint16_t target_speed = TARGET_SPEED > 0 ? TARGET_SPEED : -TARGET_SPEED;

uint16_t index_ARR_step = 1;
uint32_t n_zcr_startup = 0;
uint16_t shift_n_sqrt = 14;

uint16_t mOpenLoopBemfEvent = 0;
bool mOpenLoopBemfFailure = false;
bool mSpeedFeedbackFailure = false;

int32_t speed_sum_sp_filt = 0;
int32_t speed_sum_pot_filt = 0;
uint16_t index_pot_filt = 1;
int16_t potent_filtered = 0;

uint32_t counter_ARR_Bemf = 0;
uint64_t constant_multiplier_tmp = 0;
//}}}
cSixStep sixStep;
cPiParam piParam;
std::string gStateString = "init";

cLcd lcd;
cTraceVec mTraceVec;

//{{{
uint64_t fastSqrt (uint64_t wInput) {

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
uint16_t getPotValue() {

  uint32_t sum = 0;
  uint16_t max = 0;
  for (uint16_t i = 0; i < POT_VALUES_SIZE; i++) {
    uint16_t val = mPotValues[i];
    sum += val;
    if (val > max)
      max = val;
    }
  sum -= max;
  return sum / (POT_VALUES_SIZE - 1);
  }
//}}}

//{{{
void potSpeedTarget() {

  target_speed = sixStep.mAdcValue[1] * MAX_POT_SPEED / 4096;

  if (target_speed < MIN_POT_SPEED)
    target_speed = MIN_POT_SPEED;

  if (target_speed > (MAX_POT_SPEED / VAL_POT_SPEED_DIV))
    target_speed = (MAX_POT_SPEED/VAL_POT_SPEED_DIV);
  }
//}}}
//{{{
void potSpeed() {

  auto potMean = getPotValue();

  if (!buffer_completed) {
    speed_tmp_buffer[index_pot_filt] = potMean;
    speed_sum_pot_filt = 0;
    for (uint16_t i = 1; i <= index_pot_filt;i++)
      speed_sum_pot_filt = speed_sum_pot_filt + speed_tmp_buffer[i];
    potent_filtered = speed_sum_pot_filt / index_pot_filt;
    index_pot_filt++;

    if (index_pot_filt >= FILTER_DEEP) {
      index_pot_filt = 1;
      buffer_completed = true;
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
void speedFilter() {

  if (!array_completed) {
    speed_tmp_array[index_array] = sixStep.speed_fdbk;
    speed_sum_sp_filt = 0;
    for (uint16_t i = 1; i <= index_array;i++)
      speed_sum_sp_filt = speed_sum_sp_filt + speed_tmp_array[i];
    sixStep.speed_fdbk_filtered = speed_sum_sp_filt / index_array;
    index_array++;

    if (index_array >= FILTER_DEEP) {
     index_array = 1;
     array_completed = true;
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
void rampMotor() {

  uint32_t constant_multiplier = 100;
  uint32_t constant_multiplier_2 = 4000000000;

  if (mMotorStartupCount == 1) {
    mech_accel_hz = sixStep.ACCEL * sixStep.mNumPolePair/ 60;
    constant_multiplier_tmp = (uint64_t)constant_multiplier * (uint64_t)constant_multiplier_2;
    constant_k = constant_multiplier_tmp / (3 * mech_accel_hz);
    mcNucleoCurrentRefSetValue (sixStep.mStartupCurrent);
    Time_vector_prev_tmp = 0;
    }

  if (mMotorStartupCount < NUMBER_OF_STEPS) {
    Time_vector_tmp = ((uint64_t)1000 * (uint64_t)1000 * (uint64_t) fastSqrt(((uint64_t)mMotorStartupCount * (uint64_t)constant_k)))/632455;
    delta = Time_vector_tmp - Time_vector_prev_tmp;
    if (mMotorStartupCount == 1) {
      T_single_step_first_value = (2 * 3141) * delta / 1000;
      sixStep.ARR_value = (uint32_t)(65535);
      }
    else {
      T_single_step = (2 * 3141)* delta / 1000;
      sixStep.ARR_value = (uint32_t)(65535 * T_single_step) / T_single_step_first_value;
      }
    }
  else
    mMotorStartupCount = 1;

  if (mMotorStartupCount == 1)
    sixStep.prescaler_value = (((sixStep.SYSCLK_frequency / 1000000) * T_single_step_first_value) / 65535) - 1;

  if ((sixStep.STATUS != ALIGNMENT) && (sixStep.STATUS != START))
    mMotorStartupCount++;
  else
    Time_vector_tmp = 0;

  Time_vector_prev_tmp = Time_vector_tmp;
  }
//}}}
//{{{
void arrStep() {

  if (!sixStep.mAligning)
    sixStep.mAligning = true;

  if (sixStep.mAligned) {
    if (sixStep.VALIDATION_OK) {
      sixStep.ARR_OK = true;
      mMotorStartupCount = 1;
      index_ARR_step = 1;
      }
    else {
      sixStep.STATUS = STARTUP;
      rampMotor();
      if (index_ARR_step < sixStep.numberofitemArr) {
        hTim6.Init.Period = sixStep.ARR_value;
        hTim6.Instance->ARR = (uint32_t)hTim6.Init.Period;
        index_ARR_step++;
        }
      else if (!sixStep.ARR_OK) {
        index_ARR_step = 1;
        sixStep.ACCEL >>= 1;
        if (sixStep.ACCEL < MINIMUM_ACC)
          sixStep.ACCEL = MINIMUM_ACC;

        sixStep.STATUS = STARTUP_FAILURE;
        mcStopMotor();
        gStateString = "startup failure";
        }
      }
    }
  }
//}}}
//{{{
void arrBemf (bool up) {

  if (sixStep.mPrevStep != sixStep.mStep) {
    sixStep.mPrevStep = sixStep.mStep;

    if (sixStep.SPEED_VALIDATED) {
      if (mOpenLoopBemfEvent > BEMF_CNT_EVENT_MAX)
        mOpenLoopBemfFailure = true;

      if (up && !sixStep.BEMF_OK) {
        n_zcr_startup++;
        mOpenLoopBemfEvent = 0;
        }
      else if (!sixStep.BEMF_OK)
        mOpenLoopBemfEvent++;

      if ((n_zcr_startup >= NUMBER_ZCR) && !sixStep.BEMF_OK) {
        sixStep.BEMF_OK = true;
        n_zcr_startup = 0;
        }
      }

    if (sixStep.VALIDATION_OK) {
      counter_ARR_Bemf = __HAL_TIM_GetCounter (&hTim6);
      __HAL_TIM_SetAutoreload (&hTim6, counter_ARR_Bemf + ARR_LF / 2);

      }
    }
  }
//}}}

//{{{
void setPiParam (cPiParam* piParam) {

  piParam->Reference = sixStep.CW_CCW ? -target_speed : target_speed;

  piParam->Kp_Gain = sixStep.KP;
  piParam->Ki_Gain = sixStep.KI;

  piParam->Lower_Limit_Output = LOWER_OUT_LIMIT;
  piParam->Upper_Limit_Output = UPPER_OUT_LIMIT;

  piParam->Max_PID_Output = false;
  piParam->Min_PID_Output = false;
  }
//}}}
//{{{
int16_t piController (cPiParam* PI_PARAM, int16_t speed_fdb) {

  int32_t Error = PI_PARAM->Reference - speed_fdb;

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
    sixStep.Integral_Term_sum = KI_DIV * PI_PARAM->Upper_Limit_Output;

  if (sixStep.Integral_Term_sum < -KI_DIV * PI_PARAM->Upper_Limit_Output)
    sixStep.Integral_Term_sum = -KI_DIV * PI_PARAM->Upper_Limit_Output;

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
    if (wOutput_32 < -PI_PARAM->Upper_Limit_Output)
      wOutput_32 = -PI_PARAM->Upper_Limit_Output;
    else if (wOutput_32 > -PI_PARAM->Lower_Limit_Output)
      wOutput_32 = -PI_PARAM->Lower_Limit_Output;
    }

  return (int16_t)wOutput_32;
  }
//}}}

//{{{
void bemfDelayCalc (uint16_t piReference) {

 if (piReference >= 0) {
   if (sixStep.speed_fdbk_filtered <= 12000 && sixStep.speed_fdbk_filtered > 10000)
      sixStep.demagn_value = DEMAGN_VAL_1;
    else if (sixStep.speed_fdbk_filtered <= 10000 && sixStep.speed_fdbk_filtered > 7800)
      sixStep.demagn_value = DEMAGN_VAL_2;
    else if (sixStep.speed_fdbk_filtered <= 7800 && sixStep.speed_fdbk_filtered > 6400)
      sixStep.demagn_value = DEMAGN_VAL_3;
    else if (sixStep.speed_fdbk_filtered <= 6400 && sixStep.speed_fdbk_filtered > 5400)
      sixStep.demagn_value = DEMAGN_VAL_4;
    else if (sixStep.speed_fdbk_filtered <= 5400 && sixStep.speed_fdbk_filtered > 4650)
      sixStep.demagn_value = DEMAGN_VAL_5;
    else if (sixStep.speed_fdbk_filtered <= 4650 && sixStep.speed_fdbk_filtered > 4100)
      sixStep.demagn_value = DEMAGN_VAL_6;
    else if (sixStep.speed_fdbk_filtered <= 4100 && sixStep.speed_fdbk_filtered > 3650)
      sixStep.demagn_value = DEMAGN_VAL_7;
    else if (sixStep.speed_fdbk_filtered <= 3650 && sixStep.speed_fdbk_filtered > 3300)
      sixStep.demagn_value = DEMAGN_VAL_8;
    else if (sixStep.speed_fdbk_filtered <= 3300 && sixStep.speed_fdbk_filtered > 2600)
      sixStep.demagn_value = DEMAGN_VAL_9;
    else if (sixStep.speed_fdbk_filtered <= 2600 && sixStep.speed_fdbk_filtered > 1800)
      sixStep.demagn_value = DEMAGN_VAL_10;
    else if (sixStep.speed_fdbk_filtered <= 1800 && sixStep.speed_fdbk_filtered > 1500)
      sixStep.demagn_value = DEMAGN_VAL_11;
    else if (sixStep.speed_fdbk_filtered <= 1500 && sixStep.speed_fdbk_filtered > 1300)
      sixStep.demagn_value = DEMAGN_VAL_12;
    else if (sixStep.speed_fdbk_filtered <= 1300 && sixStep.speed_fdbk_filtered > 1000)
      sixStep.demagn_value = DEMAGN_VAL_13;
    else if (sixStep.speed_fdbk_filtered <= 1000 && sixStep.speed_fdbk_filtered > 500)
      sixStep.demagn_value = DEMAGN_VAL_14;
     }
   else {
    if (sixStep.speed_fdbk_filtered >= -12000 && sixStep.speed_fdbk_filtered < -10000)
      sixStep.demagn_value = DEMAGN_VAL_1;
    else if (sixStep.speed_fdbk_filtered >= -10000 && sixStep.speed_fdbk_filtered < -7800)
      sixStep.demagn_value = DEMAGN_VAL_2;
    else if (sixStep.speed_fdbk_filtered >= -7800 && sixStep.speed_fdbk_filtered < -6400)
      sixStep.demagn_value = DEMAGN_VAL_3;
    else if (sixStep.speed_fdbk_filtered >= -6400 && sixStep.speed_fdbk_filtered < -5400)
      sixStep.demagn_value = DEMAGN_VAL_4;
    else if (sixStep.speed_fdbk_filtered >= -5400 && sixStep.speed_fdbk_filtered < -4650)
      sixStep.demagn_value = DEMAGN_VAL_5;
    else if (sixStep.speed_fdbk_filtered >= -4650 && sixStep.speed_fdbk_filtered < -4100)
      sixStep.demagn_value = DEMAGN_VAL_6;
    else if (sixStep.speed_fdbk_filtered >= -4100 && sixStep.speed_fdbk_filtered < -3650)
      sixStep.demagn_value = DEMAGN_VAL_7;
    else if (sixStep.speed_fdbk_filtered >= -3650 && sixStep.speed_fdbk_filtered < -3300)
      sixStep.demagn_value = DEMAGN_VAL_8;
    else if (sixStep.speed_fdbk_filtered >= -3300 && sixStep.speed_fdbk_filtered < -2650)
      sixStep.demagn_value = DEMAGN_VAL_9;
    else if (sixStep.speed_fdbk_filtered >= -2600 && sixStep.speed_fdbk_filtered <-1800)
      sixStep.demagn_value = DEMAGN_VAL_10;
    else if (sixStep.speed_fdbk_filtered >= -1800 && sixStep.speed_fdbk_filtered < -1500)
      sixStep.demagn_value = DEMAGN_VAL_11;
    else if (sixStep.speed_fdbk_filtered >= -1500 && sixStep.speed_fdbk_filtered < -1300)
      sixStep.demagn_value = DEMAGN_VAL_12;
    else if (sixStep.speed_fdbk_filtered >= -1300 && sixStep.speed_fdbk_filtered < -1000)
      sixStep.demagn_value = DEMAGN_VAL_13;
    else if (sixStep.speed_fdbk_filtered >= -1000 && sixStep.speed_fdbk_filtered < -500)
      sixStep.demagn_value = DEMAGN_VAL_14;
    }
  }
//}}}
//{{{
void taskSpeed() {

  if (!sixStep.VALIDATION_OK &&
      ((sixStep.speed_fdbk_filtered > (target_speed)) || (sixStep.speed_fdbk_filtered < (-target_speed)))) {
    gStateString = "VALIDATION_OK";
    sixStep.STATUS = VALIDATION;
    sixStep.SPEED_VALIDATED = true;
    }

  if (sixStep.SPEED_VALIDATED && sixStep.BEMF_OK && !sixStep.mClosedLoopReady)
    sixStep.mClosedLoopReady = true;

  if (sixStep.VALIDATION_OK) {
    gStateString = "RUN";
    sixStep.STATUS = RUN;
    uint16_t ref = (uint16_t)piController (&piParam,(int16_t)sixStep.speed_fdbk_filtered);
    if (piParam.Reference < 0)
      ref = -ref;

    sixStep.Current_Reference = ref;
    mcNucleoCurrentRefSetValue (sixStep.Current_Reference);
    }

  bemfDelayCalc (piParam.Reference);
  }
//}}}

// callback interface
//{{{
void mcAdcSample (ADC_HandleTypeDef* adc) {

  uint16_t value = HAL_ADC_GetValue (adc);

  if (adc == &hAdc1) {
    if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) { // up
      sixStep.mBemfValue[0] = value;
      sixStep.mBemfValue[1] = 0;
      sixStep.mBemfValue[2] = 0;
      if ((sixStep.STATUS != START) && (sixStep.STATUS != ALIGNMENT))
        switch (sixStep.mStep) {
          //{{{
          case 3:
            if (sixStep.demagn_counter < sixStep.demagn_value)
              sixStep.demagn_counter++;
            else if (piParam.Reference >= 0) {
              if (value < sixStep.ADC_BEMF_threshold_DOWN)
                arrBemf (0);
              }
            else if (value > sixStep.ADC_BEMF_threshold_UP) {
              arrBemf (1);
              sixStep.BEMF_Tdown_count = 0;
              }

            break;
          //}}}
          //{{{
          case 6:
            if (sixStep.demagn_counter < sixStep.demagn_value)
              sixStep.demagn_counter++;
            else if (piParam.Reference >= 0) {
              if (value > sixStep.ADC_BEMF_threshold_UP) {
                arrBemf (1);
                sixStep.BEMF_Tdown_count = 0;
                }
              }
            else if (value < sixStep.ADC_BEMF_threshold_DOWN)
               arrBemf (0);
            break;
          //}}}
          }

      switch (sixStep.mAdcIndex) {
        case 0: mcNucleoAdcChan (&hAdc1, ADC_CHANNEL_7); break;
        case 1: mcNucleoAdcChan (&hAdc1, ADC_CHANNEL_2); break;
        case 2: mcNucleoAdcChan (&hAdc1, ADC_CHANNEL_8); break;
        }
      }
    else {
      sixStep.mAdcValue[sixStep.mAdcIndex] = value;
      sixStep.mAdcIndex = (sixStep.mAdcIndex+1) % 3;
      mcNucleoAdcChan (&hAdc1, ADC_CHANNEL_9);
      }
    }

  else if (adc == &hAdc3) {
    if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) {  // up
      sixStep.mBemfValue[0] = 0;
      sixStep.mBemfValue[1] = value;
      sixStep.mBemfValue[2] = 0;
      if ((sixStep.STATUS != START) && (sixStep.STATUS != ALIGNMENT))
        switch (sixStep.mStep) {
          //{{{
          case 2:
            if (sixStep.demagn_counter < sixStep.demagn_value)
              sixStep.demagn_counter++;
            else if (piParam.Reference >= 0) {
              if (value > sixStep.ADC_BEMF_threshold_UP) {
                arrBemf (1);
                sixStep.BEMF_Tdown_count = 0;
                }
              }
            else if (value < sixStep.ADC_BEMF_threshold_DOWN)
              arrBemf (0);

            break;
          //}}}
          //{{{
          case 5:
            if (sixStep.demagn_counter < sixStep.demagn_value)
              sixStep.demagn_counter++;
            else if (piParam.Reference >= 0) {
              if (value < sixStep.ADC_BEMF_threshold_DOWN)
                arrBemf (0);
              }
            else if (value > sixStep.ADC_BEMF_threshold_UP) {
              arrBemf (1);
              sixStep.BEMF_Tdown_count = 0;
              }

           break;
          //}}}
          }
      mcNucleoAdcChan (&hAdc3, ADC_CHANNEL_1);
      }
    else {
      sixStep.mAdcValue[3] = value;
      mPotValues[mPotValueIndex] = value;
      mPotValueIndex = (mPotValueIndex+1) % POT_VALUES_SIZE;
      mcNucleoAdcChan (&hAdc3, ADC_CHANNEL_12);
      }
    }

  else if (adc == &hAdc2) {
    if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) {  // up
      sixStep.mBemfValue[0] = 0;
      sixStep.mBemfValue[1] = 0;
      sixStep.mBemfValue[2] = value;
      if ((sixStep.STATUS != START) && (sixStep.STATUS != ALIGNMENT))
        switch (sixStep.mStep) {
          //{{{
          case 1:
            if (sixStep.demagn_counter < sixStep.demagn_value)
              sixStep.demagn_counter++;
            else if (piParam.Reference >= 0) {
              if (value < sixStep.ADC_BEMF_threshold_DOWN)
                arrBemf (0);
              }
            else if (value > sixStep.ADC_BEMF_threshold_UP) {
              arrBemf (1);
              sixStep.BEMF_Tdown_count = 0;
              }

            break;
          //}}}
          //{{{
          case 4:
            if (sixStep.demagn_counter >= sixStep.demagn_value)
              sixStep.demagn_counter++;
            else if (piParam.Reference >= 0) {
              if (value > sixStep.ADC_BEMF_threshold_UP) {
                arrBemf (1);
                sixStep.BEMF_Tdown_count = 0;
                }
              }
             else if (value < sixStep.ADC_BEMF_threshold_DOWN)
               arrBemf (0);

           break;
          //}}}
          }
      mcNucleoAdcChan (&hAdc2, ADC_CHANNEL_4);
      }
    else
      mcNucleoAdcChan (&hAdc2, ADC_CHANNEL_4);
    }
  else
    printf ("unrecognised adc\n");
  }
//}}}
//{{{
void mcTim6Tick() {

  // nextStep
  if (!sixStep.mPmwRunning) {
    mcNucleoStartPwm();
    sixStep.mPmwRunning = true;
    }
  ARR_LF = __HAL_TIM_GetAutoreload (&hTim6);

  if (sixStep.mAligned) {
    sixStep.speed_fdbk = mcGetMechSpeedRPM();
    sixStep.demagn_counter = 1;
    if (sixStep.mPrevStep != sixStep.mStep)
      n_zcr_startup = 0;

    if (piParam.Reference >= 0) {
      sixStep.mStep++;
      if (sixStep.mStep > 6)
        sixStep.mStep = 1;
      if (sixStep.mClosedLoopReady)
        sixStep.VALIDATION_OK = true;
      }
    else {
      sixStep.mStep--;
      if (sixStep.mStep < 1)
        sixStep.mStep = 6;
      if (sixStep.mClosedLoopReady)
        sixStep.VALIDATION_OK = true;
      }
    }

  if (sixStep.VALIDATION_OK) {
    // motorStall detection
    if (sixStep.BEMF_Tdown_count++ > BEMF_CONSEC_DOWN_MAX)
      mSpeedFeedbackFailure = true;
    else
      __HAL_TIM_SetAutoreload (&hTim6, 0xFFFF);
    }

  switch (sixStep.mStep) {
    case 1:
      mcNucleoSetChanCCR (sixStep.pulse_value, 0, 0);
      mcNucleoEnableInputChan12();
      if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) // step request during downCount, change adc Chan
        mcNucleoAdcChan (&hAdc2, ADC_CHANNEL_4);
      break;

    case 2:
      mcNucleoSetChanCCR (sixStep.pulse_value, 0, 0);
      mcNucleoEnableInputChan13();
      if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) // step request during downCount, change adc Chan
        mcNucleoAdcChan (&hAdc3, ADC_CHANNEL_12);
      break;

    case 3:
      mcNucleoSetChanCCR (0, sixStep.pulse_value, 0);
      mcNucleoEnableInputChan23();
      if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) // step request during downCount, change adc Chan
        mcNucleoAdcChan (&hAdc1, ADC_CHANNEL_9);
      break;

    case 4:
      mcNucleoSetChanCCR (0, sixStep.pulse_value, 0);
      mcNucleoEnableInputChan12();
      if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) // step request during downCount, change adc Chan
        mcNucleoAdcChan (&hAdc2, ADC_CHANNEL_4);
     break;

    case 5:
      mcNucleoSetChanCCR (0, 0, sixStep.pulse_value);
      mcNucleoEnableInputChan13();
      if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) // step request during downCount, change adc Chan
        mcNucleoAdcChan (&hAdc3, ADC_CHANNEL_12);
      break;

    case 6:
      mcNucleoSetChanCCR (0, 0, sixStep.pulse_value);
      mcNucleoEnableInputChan23();
      if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) // step request during downCount, change adc Chan
        mcNucleoAdcChan (&hAdc1, ADC_CHANNEL_9);
      break;
     }

  if (!sixStep.ARR_OK) // STEP frequency changing
    arrStep();

  speedFilter();
  }
//}}}
//{{{
void mcSysTick() {

  if (sixStep.mMotorRunning) {
    mTraceVec.addSample (0, sixStep.mBemfValue[0]>>4);
    mTraceVec.addSample (1, sixStep.mBemfValue[1]>>4);
    mTraceVec.addSample (2, sixStep.mBemfValue[2]>>4);
    mTraceVec.addSample (3, sixStep.mStep);
    mTraceVec.addSample (4, __HAL_TIM_GetCounter (&hTim6)>>8);
    }

  if (sixStep.mAligning && !sixStep.mAligned) {
    //{{{  align motor
    sixStep.STATUS = ALIGNMENT;
    sixStep.mStep = 6;

    hTim6.Init.Period = sixStep.ARR_value;
    hTim6.Instance->ARR = (uint32_t)hTim6.Init.Period;

    mAlignTicks++;
    if (mAlignTicks >= TIME_FOR_ALIGN + 1) {
      gStateString = "aligned";
      sixStep.mAligned = true;
      sixStep.STATUS = STARTUP;
      hTim6.Init.Prescaler = sixStep.prescaler_value;
      hTim6.Instance->PSC = hTim6.Init.Prescaler;
      }

    potSpeedTarget();
    }
    //}}}

  if (sixStep.VALIDATION_OK)
    potSpeed();
  if (sixStep.STATUS != SPEED_FEEDBACK_FAILURE)
    taskSpeed();
  if (sixStep.VALIDATION_OK)
    mcSetSpeed();

  if (mOpenLoopBemfFailure) {
    sixStep.ACCEL >>= 1;
    if (sixStep.ACCEL < MINIMUM_ACC)
      sixStep.ACCEL = MINIMUM_ACC;
    mcStopMotor();

    sixStep.STATUS = STARTUP_BEMF_FAILURE;
    gStateString = "startup Bemf failure";
    mOpenLoopBemfEvent = 0;
    }

  if (mSpeedFeedbackFailure) {
    mcStopMotor();
    sixStep.STATUS = SPEED_FEEDBACK_FAILURE;
    gStateString = "speed feedback failure";
    }
  }
//}}}

// external interface
//{{{
void mcInit() {

  mcNucleoInit();

  sixStep.HF_TIMx_ARR = hTim1.Instance->ARR;
  sixStep.HF_TIMx_PSC = hTim1.Instance->PSC;
  sixStep.HF_TIMx_CCR = hTim1.Instance->CCR1;

  sixStep.LF_TIMx_ARR = hTim6.Instance->ARR;
  sixStep.LF_TIMx_PSC = hTim6.Instance->PSC;

  sixStep.mStartupCurrent = STARTUP_CURRENT_REFERENCE;
  sixStep.mNumPolePair = NUM_POLE_PAIR;

  sixStep.ACCEL = ACC;
  sixStep.KP = KP_GAIN;
  sixStep.KI = KI_GAIN;
  sixStep.CW_CCW = TARGET_SPEED < 0;

  mcReset();
  }
//}}}
//{{{
void mcReset() {

  mLastButtonPress = 0;

  sixStep.mMotorRunning = false;
  sixStep.mPmwRunning = false;
  sixStep.mAligning = false;
  sixStep.mAligned = false;
  sixStep.VALIDATION_OK = false;
  sixStep.ARR_OK = false;
  sixStep.BEMF_OK = false;
  sixStep.mClosedLoopReady = false;
  sixStep.SPEED_VALIDATED = false;

  sixStep.numberofitemArr = NUMBER_OF_STEPS;
  sixStep.mStartupCurrent = STARTUP_CURRENT_REFERENCE;

  sixStep.pulse_value = sixStep.HF_TIMx_CCR;
  sixStep.Speed_target_ramp = MAX_POT_SPEED;
  sixStep.Speed_Ref_filtered = 0;
  sixStep.demagn_value = INITIAL_DEMAGN_DELAY;

  hTim1.Init.Prescaler = sixStep.HF_TIMx_PSC;
  hTim1.Instance->PSC =  sixStep.HF_TIMx_PSC;
  hTim1.Init.Period =    sixStep.HF_TIMx_ARR;
  hTim1.Instance->ARR =  sixStep.HF_TIMx_ARR;
  hTim1.Instance->CCR1 = sixStep.HF_TIMx_CCR;

  hTim6.Init.Prescaler = sixStep.LF_TIMx_PSC;
  hTim6.Instance->PSC =  sixStep.LF_TIMx_PSC;
  hTim6.Init.Period =    sixStep.LF_TIMx_ARR;
  hTim6.Instance->ARR =  sixStep.LF_TIMx_ARR;

  sixStep.SYSCLK_frequency = HAL_RCC_GetSysClockFreq();

  mcNucleoSetChanCCR (0,0,0);

  sixStep.ADC_BEMF_threshold_UP = BEMF_THRSLD_UP;
  sixStep.ADC_BEMF_threshold_DOWN = BEMF_THRSLD_DOWN;
  sixStep.demagn_counter = 0;

  sixStep.speed_fdbk_filtered = 0;
  sixStep.Integral_Term_sum = 0;
  sixStep.Current_Reference = 0;
  sixStep.speed_fdbk = 0;

  sixStep.BEMF_Tdown_count = 0;   // Reset of the Counter to detect Stop motor condition when a stall condition occurs

  T_single_step = 0;
  T_single_step_first_value = 0;
  Time_vector_tmp = 0;
  Time_vector_prev_tmp = 0;
  delta = 0;

  Mech_Speed_RPM = 0;
  El_Speed_Hz = 0;
  mech_accel_hz = 0;

  constant_k = 0;
  ARR_LF = 0;
  index_array = 1;

  mOpenLoopBemfEvent = 0;
  mOpenLoopBemfFailure = false;
  mSpeedFeedbackFailure = false;

  index_ARR_step = 1;
  n_zcr_startup = 0;

  mAlignTicks = 1;
  speed_sum_sp_filt = 0;
  speed_sum_pot_filt = 0;
  index_pot_filt = 1;
  potent_filtered = 0;
  counter_ARR_Bemf = 0;
  constant_multiplier_tmp = 0;

  mPotValueIndex = 0;
  for (uint16_t i = 0; i < POT_VALUES_SIZE; i++)
    mPotValues[i] = 0;

  array_completed = false;
  buffer_completed = false;
  for (uint16_t i = 0; i < FILTER_DEEP;i++) {
    speed_tmp_array[i] = 0;
    speed_tmp_buffer[i]= 0;
    }

  sixStep.mPrevStep = 0;
  if (piParam.Reference < 0)
    sixStep.mStep = 1;
  else
    sixStep.mStep = 0;

  target_speed = TARGET_SPEED;
  setPiParam (&piParam);

  mcNucleoCurrentRefStart();
  mcNucleoCurrentRefSetValue (sixStep.mStartupCurrent);

  mMotorStartupCount = 1;
  rampMotor();
  }
//}}}

//{{{
int32_t mcGetElSpeedHz() {

  if (__HAL_TIM_GetAutoreload (&hTim6) != 0xFFFF)
    El_Speed_Hz = (int32_t)((sixStep.SYSCLK_frequency) / hTim6.Instance->PSC) / (__HAL_TIM_GetAutoreload (&hTim6) * 6);
  else
    El_Speed_Hz = 0;

  return piParam.Reference < 0 ? -El_Speed_Hz : El_Speed_Hz;
  }
//}}}
//{{{
int32_t mcGetMechSpeedRPM() {

  Mech_Speed_RPM = (int32_t)(mcGetElSpeedHz() *  60 / sixStep.mNumPolePair);
  return (Mech_Speed_RPM);
  }
//}}}

//{{{
void mcStartMotor() {

  sixStep.STATUS = START;
  sixStep.mMotorRunning = true;

  HAL_TIM_Base_Start_IT (&hTim6);
  HAL_ADC_Start_IT (&hAdc1);
  HAL_ADC_Start_IT (&hAdc2);
  HAL_ADC_Start_IT (&hAdc3);

  mcNucleoLedOn();
  }
//}}}
//{{{
void mcStopMotor() {

  sixStep.STATUS = STOP;
  sixStep.mMotorRunning = false;

  mcNucleoStopPwm();
  hTim1.Instance->CR1 &= ~(TIM_CR1_CEN);
  hTim1.Instance->CNT = 0;
  mcNucleoDisableChan();

  HAL_TIM_Base_Stop_IT (&hTim6);
  HAL_ADC_Stop_IT (&hAdc1);
  HAL_ADC_Stop_IT (&hAdc2);
  HAL_ADC_Stop_IT (&hAdc3);

  mcNucleoCurrentRefStop();
  mcNucleoLedOff();

  mcReset();
  }
//}}}
//{{{
void mcPanic() {

  mcStopMotor();
  sixStep.STATUS = OVERCURRENT_FAILURE;
  gStateString = "mcPanic";
  }
//}}}

//{{{
void mcSetSpeed() {

  int16_t reference_tmp = 0;

  bool change_target_speed = false;
  if (sixStep.Speed_Ref_filtered > sixStep.Speed_target_ramp) {
    if ((sixStep.Speed_Ref_filtered - sixStep.Speed_target_ramp) > ADC_SPEED_TH)
      change_target_speed = true;
    }
  else if ((sixStep.Speed_target_ramp - sixStep.Speed_Ref_filtered) > ADC_SPEED_TH)
    change_target_speed = true;

  if (change_target_speed) {
    sixStep.Speed_target_ramp = sixStep.Speed_Ref_filtered;

    if (sixStep.CW_CCW) {
      reference_tmp = -(sixStep.Speed_Ref_filtered * MAX_POT_SPEED / 4096);
      piParam.Reference = (reference_tmp >=- MIN_POT_SPEED) ? -MIN_POT_SPEED : reference_tmp;
      }
    else {
      reference_tmp = sixStep.Speed_Ref_filtered * MAX_POT_SPEED / 4096;
      piParam.Reference = (reference_tmp <= MIN_POT_SPEED) ? MIN_POT_SPEED : reference_tmp;
      }
    }
  }
//}}}
//{{{
void mcEXTbutton() {

  if (HAL_GetTick() > mLastButtonPress + 200) {
    mLastButtonPress = HAL_GetTick();
    if (sixStep.mMotorRunning) {
      printf ("StopMotor\n");
      mcStopMotor();
      gStateString = "stopMotor";
      }
    else {
      printf ("StartMotor\n");
      mcStartMotor();
      gStateString = "startMotor";
      }
    }
  }
//}}}

// callbacks
//{{{
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc) {
  mcAdcSample (hadc);
  }
//}}}
//{{{
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef* htim) {
  if (htim == &hTim6)
    mcTim6Tick();
  }
//}}}
//{{{
void HAL_SYSTICK_Callback() {
  mcSysTick();
  }
//}}}
//{{{
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {
  mcEXTbutton();
  }
//}}}

//{{{
void systemClockConfig() {

  // config osc
  RCC_OscInitTypeDef rccOscInitStruct;
  rccOscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  rccOscInitStruct.HSEState = RCC_HSE_ON;
  rccOscInitStruct.HSIState = RCC_HSI_ON;
  rccOscInitStruct.PLL.PLLState = RCC_PLL_ON;
  rccOscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  rccOscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  rccOscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig (&rccOscInitStruct) != HAL_OK)
    printf ("HAL_RCC_OscConfig failed\n");

  // config CPU, AHB and APB clocks
  RCC_ClkInitTypeDef rccClkInitStruct;
  rccClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  rccClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  rccClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  rccClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  rccClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig (&rccClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    printf ("HAL_RCC_ClockConfig failed\n");

  // config periph clocks
  RCC_PeriphCLKInitTypeDef periphClkInit;
  periphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_ADC34 |
                                       RCC_PERIPHCLK_TIM1 | RCC_PERIPHCLK_TIM16;
  periphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  periphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  periphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  periphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig (&periphClkInit) != HAL_OK)
    printf ("HAL_RCCEx_PeriphCLKConfig failed\n");

  // enable Clock Security System
  HAL_RCC_EnableCSS();

  // config Systick interrupt time
  HAL_SYSTICK_Config (HAL_RCC_GetHCLKFreq() / 1000);

  // config Systick
  HAL_SYSTICK_CLKSourceConfig (SYSTICK_CLKSOURCE_HCLK);

  // SysTick_IRQn interrupt configuration
  HAL_NVIC_SetPriority (SysTick_IRQn, 4, 0);
  }
//}}}
//{{{
int main() {

  HAL_Init();
  systemClockConfig();

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping (NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority (MemoryManagement_IRQn, 0, 0);
  HAL_NVIC_SetPriority (BusFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority (UsageFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority (SVCall_IRQn, 0, 0);
  HAL_NVIC_SetPriority (DebugMonitor_IRQn, 0, 0);
  HAL_NVIC_SetPriority (PendSV_IRQn, 0, 0);
  HAL_NVIC_SetPriority (SysTick_IRQn, 2, 0);

  mcInit();

  lcd.init();
  mTraceVec.addTrace (1000, 1, 5);

  while (1) {
    lcd.clear (cLcd::eOn);
    //printf ("%d %d %d i:%d p:%d v:%d t:%d 1:%d 2:%d 3:%d\n",
    //        ARR_LF, counter_ARR_Bemf, piParam.Reference,
    //        sixStep.mAdcBuffer[0], sixStep.mAdcBuffer[1] ,sixStep.mAdcBuffer[2] ,sixStep.mAdcBuffer[3],
    //        sixStep.mBemfValue[0], sixStep.mBemfValue[1] ,sixStep.mBemfValue[2]);

    lcd.drawString (cLcd::eOff, cLcd::eSmall, cLcd::eLeft,
                    "cur:" + dec (sixStep.mAdcValue[0], 4) +
                    " vbus:" + dec (sixStep.mAdcValue[1], 4) +
                    " temp:" + dec (sixStep.mAdcValue[2], 4) +
                    " pot:" + dec (sixStep.mAdcValue[3], 4),
                    cPoint(0,0));

    lcd.drawString (cLcd::eOff, cLcd::eSmall, cLcd::eLeft,
                    dec (sixStep.mBemfValue[0], 4) + " " +
                    dec (sixStep.mBemfValue[1], 4) + " " +
                    dec (sixStep.mBemfValue[2], 4),
                    cPoint(0,20));

    lcd.drawString (cLcd::eOff, cLcd::eSmall, cLcd::eLeft, gStateString, cPoint(0,40));

    mTraceVec.draw (&lcd, 60, lcd.getHeight());
    lcd.present();
    }
  }
//}}}
