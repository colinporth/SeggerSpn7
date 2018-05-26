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
class sSixStep {
public:
  eSixStepStatus STATUS = IDLE;        // Status variable for SixStep algorithm

  bool Button_ready = false;           //
  bool CMD = false;                    // Flag control for Motor Start/Stop
  bool RUN_Motor = false;              // Flag for Motor status
  bool ALIGNMENT = false;              // Flag control for Motor Alignment ongoing
  bool ALIGN_OK = false;               // Flag control for Motor Alignment
  bool ARR_OK = false;                 // ARR flag control for Accell status
  bool SPEED_VALIDATED = false;        // Validation flag for Speed before closed loop control
  bool VALIDATION_OK = false;          // Validation flag for Closed loop control begin
  bool BEMF_OK = false;                //
  bool CL_READY = false;               //

  bool CW_CCW = false;                 // Set the motor direction

  uint32_t LF_TIMx_PSC = 0;            // Prescaler variable for low frequency timer
  uint32_t LF_TIMx_ARR = 0;            // ARR variable for low frequency timer
  uint32_t HF_TIMx_PSC = 0;            // Prescaler variable for high frequency timer
  uint32_t HF_TIMx_ARR = 0;            // ARR variable for high frequency timer
  uint32_t HF_TIMx_CCR = 0;            // CCR variable for high frequency timer

  uint8_t step_position = 0;           // Step number for SixStep algorithm
  uint8_t prev_step_position = 0;      // Previous step number for SixStep algorithm
  uint16_t pulse_value = 0;            // CCR value for SixStep algorithm
  uint16_t ARR_value = 0;              // ARR vector for Accell compute
  uint32_t Regular_channel[4];         // Buffer for ADC regular channel
  uint32_t CurrentRegular_BEMF_ch = 0; // ADC regular channel to select
  uint32_t prescaler_value = 0;        // Prescaler value for low freq timer
  uint16_t numberofitemArr = 0;        // Number of elements

  uint32_t ADC_BUFFER[4];              // Buffer for ADC regular channel
  uint32_t ADC_SEQ_CHANNEL[4];         // Buffer for ADC regular channel
  uint32_t ADC_Regular_Buffer[5];      // Buffer for ADC regular channel
  uint16_t ADC_BEMF_threshold_UP = 0;  // Voltage threshold for BEMF detection in up direction
  uint16_t ADC_BEMF_threshold_DOWN = 0;// Voltage threshold for BEMF detection in down direction

  uint16_t demagn_counter = 0;         // Demagnetization counter
  uint16_t demagn_value = 0;           // Demagnetization value

  int16_t speed_fdbk = 0;              // Motor speed variable
  int16_t speed_fdbk_filtered = 0;     // Filtered Motor speed variable
  int16_t filter_depth = 0;            // Filter depth for speed measuring

  uint16_t Current_Reference = 0;      // Currrent reference for SixStep algorithm
  uint16_t Ireference = 0;             // Currrent reference for SixStep algorithm
  int32_t Integral_Term_sum = 0;       // Global Integral part for PI
  uint8_t bemf_state_1 = 0;            // Bemf variable
  uint8_t bemf_state_2 = 0;            // Bemf variable
  uint8_t bemf_state_3 = 0;            // Bemf variable
  uint8_t bemf_state_4 = 0;            // Bemf variable
  uint8_t bemf_state_5 = 0;            // Bemf variable
  uint8_t bemf_state_6 = 0;            // Bemf variable

  uint16_t Speed_Loop_Time = 0;        // Speed loop variable for timing
  uint16_t Speed_Ref_filtered = 0;     // Filtered Reference Motor Speed variable
  uint16_t Speed_target_ramp = 0;      // Target Motor Speed
  uint16_t Speed_target_time = 0;      // Target Motor Ramp time

  uint16_t Ramp_Start = 0;             // Ramp time start
  uint16_t Bemf_delay_start = 0;       // Bemf variable

  uint32_t SYSCLK_frequency = 0;       // System clock main frequency
  uint8_t BEMF_Tdown_count = 0;        // BEMF Consecutive Threshold Falling Crossings Counter
  uint16_t IREFERENCE = 0;             // Currrent reference
  uint16_t NUMPOLESPAIRS = 0;          // Number of motor pole pairs
  uint32_t ACCEL = 0;                  // Acceleration start-up parameter
  uint16_t KP = 0;                     // KP parameter for PI regulator
  uint16_t KI = 0;                     // KI parameter for PI regulator
  };
//}}}
//{{{
class sPiParam {
public:
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
