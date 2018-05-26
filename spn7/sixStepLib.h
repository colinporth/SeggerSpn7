#pragma once

#include "ihm07m1.h"
#include "sixStepParam.h"
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

enum eSixStepStatus {
  IDLE, STARTUP, VALIDATION, STOP, START, RUN, ALIGNMENT,
  SPEEDFBKERROR, OVERCURRENT, STARTUP_FAILURE, STARTUP_BEMF_FAILURE };

//{{{
struct sSixStep {
  eSixStepStatus STATUS;               // Status variable for SixStep algorithm

  bool Button_ready;                   //
  bool CMD;                            // Flag control for Motor Start/Stop
  bool RUN_Motor;                      // Flag for Motor status
  bool ALIGNMENT;                      // Flag control for Motor Alignment ongoing
  bool ALIGN_OK;                       // Flag control for Motor Alignment
  bool ARR_OK;                         // ARR flag control for Accell status
  bool SPEED_VALIDATED;                // Validation flag for Speed before closed loop control
  bool VALIDATION_OK;                  // Validation flag for Closed loop control begin
  bool BEMF_OK;                        //
  bool CL_READY;                       //

  bool CW_CCW;                         // Set the motor direction

  uint32_t LF_TIMx_PSC;                // Prescaler variable for low frequency timer
  uint32_t LF_TIMx_ARR;                // ARR variable for low frequency timer
  uint32_t HF_TIMx_PSC;                // Prescaler variable for high frequency timer
  uint32_t HF_TIMx_ARR;                // ARR variable for high frequency timer
  uint32_t HF_TIMx_CCR;                // CCR variable for high frequency timer

  uint8_t step_position;               // Step number for SixStep algorithm
  uint8_t prev_step_position;          // Previous step number for SixStep algorithm
  uint16_t pulse_value;                // CCR value for SixStep algorithm
  uint16_t ARR_value;                  // ARR vector for Accell compute
  uint32_t Regular_channel[4];         // Buffer for ADC regular channel
  uint32_t CurrentRegular_BEMF_ch;     // ADC regular channel to select
  uint32_t prescaler_value;            // Prescaler value for low freq timer
  uint16_t numberofitemArr;            // Number of elements

  uint32_t ADC_BUFFER[4];              // Buffer for ADC regular channel
  uint32_t ADC_SEQ_CHANNEL[4];         // Buffer for ADC regular channel
  uint32_t ADC_Regular_Buffer[5];      // Buffer for ADC regular channel
  uint16_t ADC_BEMF_threshold_UP;      // Voltage threshold for BEMF detection in up direction
  uint16_t ADC_BEMF_threshold_DOWN;    // Voltage threshold for BEMF detection in down direction

  uint16_t demagn_counter;             // Demagnetization counter
  uint16_t demagn_value;               // Demagnetization value

  int16_t speed_fdbk;                  // Motor speed variable
  int16_t speed_fdbk_filtered;         // Filtered Motor speed variable
  int16_t filter_depth;                // Filter depth for speed measuring

  uint16_t Current_Reference;          // Currrent reference for SixStep algorithm
  uint16_t Ireference;                 // Currrent reference for SixStep algorithm
  int32_t Integral_Term_sum;           // Global Integral part for PI
  uint8_t bemf_state_1;                // Bemf variable
  uint8_t bemf_state_2;                // Bemf variable
  uint8_t bemf_state_3;                // Bemf variable
  uint8_t bemf_state_4;                // Bemf variable
  uint8_t bemf_state_5;                // Bemf variable
  uint8_t bemf_state_6;                // Bemf variable

  uint16_t Speed_Loop_Time;            // Speed loop variable for timing
  uint16_t Speed_Ref_filtered;         // Filtered Reference Motor Speed variable
  uint16_t Speed_target_ramp;          // Target Motor Speed
  uint16_t Speed_target_time;          // Target Motor Ramp time

  uint16_t Ramp_Start;                 // Ramp time start
  uint16_t Bemf_delay_start;           // Bemf variable

  uint32_t SYSCLK_frequency;           // System clock main frequency
  uint8_t BEMF_Tdown_count;            // BEMF Consecutive Threshold Falling Crossings Counter
  uint16_t IREFERENCE;                 // Currrent reference
  uint16_t NUMPOLESPAIRS;              // Number of motor pole pairs
  uint32_t ACCEL;                      // Acceleration start-up parameter
  uint16_t KP;                         // KP parameter for PI regulator
  uint16_t KI;                         // KI parameter for PI regulator
  };
//}}}
//{{{
struct sPiParam {
  int16_t Reference;          // Refence value for PI regulator
  int16_t Kp_Gain;            // Kp value for PI regulator
  int16_t Ki_Gain;            // Ki value for PI regulator
  int16_t Lower_Limit_Output; // Min output value for PI regulator
  int16_t Upper_Limit_Output; // Max output value for PI regulator
  bool Max_PID_Output;        // Max Saturation indicator flag
  bool Min_PID_Output;        // Min Saturation indicator flag
  };
//}}}

// external interface
void MC_Init();
void MC_Reset();

int32_t MC_GetElSpeedHz();
int32_t MC_GetMechSpeedRPM();

void MC_StartMotor();
void MC_StopMotor();

void MC_SetSpeed();
void MC_EXTbutton();

//{{{
#ifdef __cplusplus
}
#endif
//}}}
