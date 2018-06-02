#pragma once
#include "sixStepParam.h"
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

class cSixStep {
public:
  enum eSixStepStatus { STARTUP_BEMF_FAIL, OVERCURRENT_FAIL, SPEED_FEEDBACK_FAIL, STARTUP_FAIL,
                        STOPPED, START, ALIGN, STARTUP, VALIDATION, SPEED_OK, BEMF_OK, RUN };
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

  void init();
  void reset();

  void startMotor();
  void stopMotor (eSixStepStatus status);

  int32_t getSpeedRPM();
  void setSpeed();

  void adcSample (ADC_HandleTypeDef* hadc);
  void tim6Tick();
  void sysTick();

  // state
  eSixStepStatus mStatus = STOPPED;  // Status variable for SixStep algorithm

  uint16_t mAdcValue[4];          // chan 0-3 lastReadValue
  int16_t mSpeedMeasured = 0;     // measured speed
  int16_t mSpeedFiltered = 0;     // filtered speed
  int16_t mSpeedRef = 0;          // reference speed

private:
  //{{{  methods
  void GPIO_Init();
  void ADC_Init();
  void TIM1_Init();
  void TIM6_Init();
  void TIM16_Init();
  void mcNucleoDisableChan();
  void mcNucleoEnableInputChan12();
  void mcNucleoEnableInputChan13();
  void mcNucleoEnableInputChan23();
  void mcNucleoSetChanCCR (uint16_t value1, uint16_t value2, uint16_t value3);
  void mcNucleoStartPwm();
  void mcNucleoStopPwm();
  void mcNucleoCurrentRefStart();
  void mcNucleoCurrentRefStop();
  void mcNucleoCurrentRefSetValue (uint16_t value);
  void mcNucleoAdcChan (ADC_HandleTypeDef* adc, uint32_t chan);
  void mcNucleoLedOn();
  void mcNucleoLedOff();
  void mcNucleoInit();

  uint64_t fastSqrt (uint64_t input);
  void rampMotor();
  void arrBemf (bool up);
  void setPiParam (cPiParam* piParam);
  int16_t piController (cPiParam* PI_PARAM, int16_t speed_fdb);
  uint16_t getDemagnValue (uint16_t piReference);
  void taskSpeed();
  //}}}
  //{{{  vars
  cPiParam piParam;

  bool ARR_OK = false;             // ARR flag control for Accell status

  // init from params
  bool CW_CCW = false;                                  // Set the motor direction
  const uint16_t mNumPolePair = NUM_POLE_PAIR;                // Number of motor pole pairs
  const uint16_t mStartupCurrent = STARTUP_CURRENT_REFERENCE; // Currrent reference
  const uint16_t mBemfUpThreshold = BEMF_THRSLD_UP;           // Voltage threshold for BEMF detection in up direction
  const uint16_t mBemfDownThreshold = BEMF_THRSLD_DOWN;       // Voltage threshold for BEMF detection in down direction

  const uint16_t KP = KP_GAIN;    // KP parameter for PI regulator
  const uint16_t KI = KI_GAIN;    // KI parameter for PI regulator

  uint32_t ACCEL = ACC;           // Acceleration start-up parameter

  // unchanging values
  uint32_t mSysClkFrequency = 0;  // System clock main frequency

  // odd reg addresses
  uint32_t HF_TIMx_PSC = 0;       // Prescaler variable for high frequency timer
  uint32_t HF_TIMx_ARR = 0;       // ARR variable for high frequency timer
  uint32_t HF_TIMx_CCR = 0;       // CCR variable for high frequency timer
  uint32_t LF_TIMx_PSC = 0;       // Prescaler variable for low frequency timer
  uint32_t LF_TIMx_ARR = 0;       // ARR variable for low frequency timer

  int16_t mStep = -1;
  int16_t mPrevStep = -1;

  uint16_t mPulseValue = 0;       // CCR value for SixStep algorithm
  uint16_t mArrValue = 0;         // ARR vector for Accell compute
  uint32_t prescaler_value = 0;   // Prescaler value for low freq timer
  uint16_t numberofitemArr = 0;   // Number of elements

  uint16_t mAdcIndex = 0;         // adc current/pot/vbus/temp index
  uint16_t mDemagnCounter = 0;    // Demagnetization counter
  uint16_t mDemagnValue = 0;      // Demagnetization value

  uint16_t mCurrentReference = 0; // Currrent reference for SixStep algorithm
  uint16_t mAlignTicks = 1;
  uint16_t mStartupStepCount = 0;
  uint16_t mSpeedTargetRamp = 0;  // Target Motor Speed
  uint8_t mBemfDownCount = 0;     // BEMF Consecutive Threshold Falling Crossings Counter
  int32_t mIntegralTermSum = 0;   // Global Integral part for PI

  uint32_t constant_k = 0;
  uint32_t mTimeVectorPrev = 0 ;
  uint32_t mFirstSingleStep = 0;
  uint16_t mTargetSpeed = 0;
  uint32_t mArrLF = 0;
  uint16_t index_ARR_step = 1;
  uint32_t mNumZeroCrossing = 0;

  int16_t mPotArray[32];
  uint16_t mPotArrayValues = 0;
  uint16_t mPotArrayIndex = 0;

  int16_t mSpeedArray[32];
  uint16_t mSpeedArrayValues = 0;
  uint16_t mSpeedArrayIndex = 0;

  uint16_t mOpenLoopBemfEvent = 0;
  bool mOpenLoopBemfFail = false;
  bool mSpeedMeasuredFail = false;
  //}}}
  };

//{{{
#ifdef __cplusplus
}
#endif
//}}}
