#pragma once
#include "sixStepParam.h"
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

enum eSixStepStatus { IDLE, STARTUP, ALIGNMENT, VALIDATION, START, RUN, STOP,
                      STARTUP_BEMF_FAIL, OVERCURRENT_FAIL, SPEED_FEEDBACK_FAIL, STARTUP_FAIL };
//{{{
class cSixStep {
public:
  // states
  eSixStepStatus STATUS = IDLE;        // Status variable for SixStep algorithm

  bool mMotorRunning = false;
  bool mPmwRunning  = false;
  bool mAligning = false;
  bool mAligned = false;
  bool ARR_OK = false;                 // ARR flag control for Accell status
  bool SPEED_VALIDATED = false;        // Validation flag for Speed before closed loop control
  bool VALIDATION_OK = false;          // Validation flag for Closed loop control begin
  bool BEMF_OK = false;
  bool mClosedLoopReady = false;

  // init from parameters
  bool CW_CCW = false;             // Set the motor direction
  uint16_t mStartupCurrent = 0;    // Currrent reference
  uint16_t mNumPolePair = 0;       // Number of motor pole pairs
  uint16_t mBemfUpThreshold = 0;   // Voltage threshold for BEMF detection in up direction
  uint16_t mBemfDownThreshold = 0; // Voltage threshold for BEMF detection in down direction

  // unchanging values
  uint32_t SYSCLK_frequency = 0;  // System clock main frequency

  // odd reg addresses
  uint32_t HF_TIMx_PSC = 0;       // Prescaler variable for high frequency timer
  uint32_t HF_TIMx_ARR = 0;       // ARR variable for high frequency timer
  uint32_t HF_TIMx_CCR = 0;       // CCR variable for high frequency timer
  uint32_t LF_TIMx_PSC = 0;       // Prescaler variable for low frequency timer
  uint32_t LF_TIMx_ARR = 0;       // ARR variable for low frequency timer

  int16_t mStep = -1;
  int16_t mPrevStep = -1;

  uint16_t pulse_value = 0;       // CCR value for SixStep algorithm
  uint16_t ARR_value = 0;         // ARR vector for Accell compute
  uint32_t prescaler_value = 0;   // Prescaler value for low freq timer
  uint16_t numberofitemArr = 0;   // Number of elements

  uint16_t mAdcIndex = 0;         // adc current/pot/vbus/temp index
  uint16_t mAdcValue[4];          // chan 0-3 lastReadValue

  uint16_t mDemagnCounter = 0;    // Demagnetization counter
  uint16_t mDemagnValue = 0;      // Demagnetization value

  int16_t mSpeedMeasured = 0;     // measured speed
  int16_t mSpeedFiltered = 0;     // filtered speed
  int16_t mSpeedRef = 0;          // reference speed

  uint16_t mCurrentReference = 0; // Currrent reference for SixStep algorithm

  int32_t Integral_Term_sum = 0;  // Global Integral part for PI

  uint16_t Speed_target_ramp = 0; // Target Motor Speed
  uint16_t Speed_target_time = 0; // Target Motor Ramp time

  uint16_t Bemf_delay_start = 0;  // Bemf variable
  uint8_t BEMF_Tdown_count = 0;   // BEMF Consecutive Threshold Falling Crossings Counter

  uint32_t ACCEL = 0;             // Acceleration start-up parameter
  uint16_t KP = 0;                // KP parameter for PI regulator
  uint16_t KI = 0;                // KI parameter for PI regulator
  };
//}}}
//{{{
class cPiParam {
public:
  int16_t Reference;          // refence value for PI regulator

  int16_t Kp_Gain;            // Kp value for PI regulator
  int16_t Ki_Gain;            // Ki value for PI regulator

  int16_t Lower_Limit_Output; // min output value for PI regulator
  int16_t Upper_Limit_Output; // max output value for PI regulator

  bool Max_PID_Output;        // max saturation indicator flag
  bool Min_PID_Output;        // min saturation indicator flag
  };
//}}}

// external interface
void mcInit();
void mcReset();

int32_t mcGetSpeedRPM();

void mcStartMotor();
void mcStopMotor();
void mcPanic();

void mcSetSpeed();
void mcEXTbutton();

//{{{
#ifdef __cplusplus
}
#endif
//}}}
