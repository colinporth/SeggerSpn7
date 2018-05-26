#pragma once

#include "ihm07m1.h"
#include "sixStepParam.h"
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

//{{{  enum SIXSTEP_Base_SystStatus_t;
typedef enum {
  IDLE,                               /* 0 */
  STARTUP,                            /* 1 */
  VALIDATION,                         /* 2 */
  STOP,                               /* 3 */
  START,                              /* 4 */
  RUN,                                /* 5 */
  ALIGNMENT,                          /* 6 */
  SPEEDFBKERROR,                      /* 7 */
  OVERCURRENT,                        /* 8 */
  STARTUP_FAILURE,                    /* 9 */
  STARTUP_BEMF_FAILURE                /* 10 */
  } SIXSTEP_Base_SystStatus_t;
//}}}

//{{{  struct SIXSTEP_Base_InitTypeDef
typedef struct {
  uint32_t LF_TIMx_PSC;                  /*!< Prescaler variable for low frequency timer*/
  uint32_t LF_TIMx_ARR;                  /*!< ARR variable for low frequency timer*/
  uint32_t HF_TIMx_PSC;                  /*!< Prescaler variable for high frequency timer*/
  uint32_t HF_TIMx_ARR;                  /*!< ARR variable for high frequency timer*/
  uint32_t HF_TIMx_CCR;                  /*!< CCR variable for high frequency timer*/
  uint8_t step_position;                 /*!< Step number variable for SixStep algorithm*/
  SIXSTEP_Base_SystStatus_t STATUS;      /*!< Status variable for SixStep algorithm*/
  uint8_t  status_prev;                  /*!< Previous status variable for SixStep algorithm*/
  uint16_t pulse_value;                  /*!< CCR value for SixStep algorithm*/
  uint16_t ARR_value;                    /*!< ARR vector for Accell compute*/
  uint32_t Regular_channel[4];           /*!< Buffer for ADC regular channel */
  uint32_t CurrentRegular_BEMF_ch;       /*!< ADC regular channel to select */
  uint32_t prescaler_value;              /*!< Prescaler value for low freq timer*/
  uint16_t numberofitemArr;              /*!< Number of elements */
  uint32_t ADC_BUFFER[4];                /*!< Buffer for ADC regular channel */
  uint32_t ADC_SEQ_CHANNEL[4];           /*!< Buffer for ADC regular channel */
  uint32_t ADC_Regular_Buffer[5];        /*!< Buffer for ADC regular channel */
  uint16_t ADC_BEMF_threshold_UP;        /*!< Voltage threshold for BEMF detection in up direction*/
  uint16_t ADC_BEMF_threshold_DOWN;      /*!< Voltage threshold for BEMF detection in down direction*/
  uint16_t demagn_counter;               /*!< Demagnetization counter*/
  uint16_t demagn_value;                 /*!< Demagnetization value*/
  int16_t speed_fdbk;                    /*!< Motor speed variable*/
  int16_t speed_fdbk_filtered;           /*!< Filtered Motor speed variable*/
  int16_t filter_depth;                  /*!< Filter depth for speed measuring*/
  uint16_t Current_Reference;            /*!< Currrent reference for SixStep algorithm*/
  uint16_t Ireference;                   /*!< Currrent reference for SixStep algorithm*/
  int32_t Integral_Term_sum;             /*!< Global Integral part for PI*/
  uint8_t CMD;                           /*!< Flag control for Motor Start/Stop*/
  uint8_t ALIGN_OK;                      /*!< Flag control for Motor Alignment*/
  uint8_t ALIGNMENT;                     /*!< Flag control for Motor Alignment ongoing*/
  uint8_t bemf_state_1;                  /*!< Bemf variable */
  uint8_t bemf_state_2;                  /*!< Bemf variable */
  uint8_t bemf_state_3;                  /*!< Bemf variable */
  uint8_t bemf_state_4;                  /*!< Bemf variable */
  uint8_t bemf_state_5;                  /*!< Bemf variable */
  uint8_t bemf_state_6;                  /*!< Bemf variable */
  uint16_t Speed_Loop_Time;              /*!< Speed loop variable for timing */
  uint16_t Speed_Ref_filtered;           /*!< Filtered Reference Motor Speed variable */
  uint16_t RUN_Motor;                    /*!< Flag for Motor status */
  uint8_t ARR_OK;                        /*!< ARR flag control for Accell status */
  uint8_t VALIDATION_OK;                 /*!< Validation flag for Closed loop control begin */
  uint8_t SPEED_VALIDATED;               /*!< Validation flag for Speed before closed loop control */
  uint16_t Speed_target_ramp;            /*!< Target Motor Speed */
  uint16_t Speed_target_time;            /*!< Target Motor Ramp time */
  uint16_t Ramp_Start;                   /*!< Ramp time start*/
  uint16_t Bemf_delay_start;             /*!< Bemf variable */
  uint16_t MediumFrequencyTask_flag;     /*!< Flag for Medium Task Frequency */
  uint32_t SYSCLK_frequency;             /*!< System clock main frequency */
  uint32_t Uart_cmd_to_set;              /*!<  */
  uint32_t Uart_value_to_set;            /*!<  */
  uint8_t Button_ready;                  /*!<  */
  uint8_t BEMF_OK;                       /*!<  */
  uint8_t CL_READY;                      /*!<  */
  uint8_t BEMF_Tdown_count;              /*!< BEMF Consecutive Threshold Falling Crossings Counter */
  uint16_t IREFERENCE;                   /*!< Currrent reference*/
  uint16_t NUMPOLESPAIRS;                /*!< Number of motor pole pairs  */
  uint32_t ACCEL;                        /*!< Acceleration start-up parameter*/
  uint16_t KP;                           /*!< KP parameter for PI regulator */
  uint16_t KI;                           /*!< KI parameter for PI regulator */
  uint8_t CW_CCW;                        /*!< Set the motor direction */
  } SIXSTEP_Base_InitTypeDef;             /*!< Six Step Data Structure */
//}}}
//{{{  struct SIXSTEP_PI_PARAM_InitTypeDef_t
typedef struct {
  int16_t Reference;                    /*!< Refence value for PI regulator */
  int16_t Kp_Gain;                      /*!< Kp value for PI regulator */
  int16_t Ki_Gain;                      /*!< Ki value for PI regulator */
  int16_t Lower_Limit_Output;           /*!< Min output value for PI regulator */
  int16_t Upper_Limit_Output;           /*!< Max output value for PI regulator */
  int8_t Max_PID_Output;                /*!< Max Saturation indicator flag */
  int8_t Min_PID_Output;                /*!< Min Saturation indicator flag */
  } SIXSTEP_PI_PARAM_InitTypeDef_t, *SIXSTEP_pi_PARAM_InitTypeDef_t;  /*!< PI Data Structure */
//}}}

// external interface
void MC_Init();
void MC_Reset();

int32_t MC_GetElSpeedHz();
int32_t MC_GetMechSpeedRPM();

void MC_StartMotor();
void MC_StopMotor();

void MC_SetSpeed (uint16_t);
void MC_EXTbutton();

//{{{
#ifdef __cplusplus
}
#endif
//}}}
