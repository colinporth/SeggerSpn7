//{{{  includes
#include "pid_regulator.h"
#include "digital_output.h"
#include "mc_type.h"
#include "mc_math.h"
#include "mc_irq_handler.h"
#include "mc_config.h"
#include "state_machine.h"
#include "mc_api.h"
#include "pwm_common.h"
#include "circle_limitation.h"
#include "pwm_curr_fdbk.h"
#include "r3_4_f30x_pwm_curr_fdbk.h"
#include "revup_ctrl.h"
#include "bus_voltage_sensor.h"
#include "r_divider_bus_voltage_sensor.h"
#include "virtual_bus_voltage_sensor.h"
#include "ntc_temperature_sensor.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "ramp_ext_mngr.h"
#include "SystemNDriveParams.h"

#include "mc_tasks.h"
#include "mc_extended_api.h"
//}}}
//{{{  defines
#define CHARGE_BOOT_CAP_MS  10
#define CHARGE_BOOT_CAP_MS2 10
#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0
#define STOPPERMANENCY_MS  400
#define STOPPERMANENCY_MS2 400
#define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS)/ 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS2)/ 1000)
#define OFFCALIBRWAITTICKS     (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000)
#define OFFCALIBRWAITTICKS2    (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000)
#define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)/ 1000)
#define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2)/ 1000)
//}}}
//{{{  vars
FOCVars_t FOCVars[NBR_OF_MOTORS];
MCI_Handle_t Mci[NBR_OF_MOTORS];
MCI_Handle_t * oMCInterface[NBR_OF_MOTORS];
MCT_Handle_t MCT[NBR_OF_MOTORS];
STM_Handle_t STM[NBR_OF_MOTORS];
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS];
PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
PID_Handle_t *pPIDId[NBR_OF_MOTORS];
RDivider_Handle_t *pBusSensorM1;
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS];
PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];
DOUT_handle_t *pR_Brake[NBR_OF_MOTORS];
DOUT_handle_t *pOCPDisabling[NBR_OF_MOTORS];
PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS];
CircleLimitation_Handle_t *pCLM[NBR_OF_MOTORS];
RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS];   /*!< Ramp manager used to modify the Iq ref
																										during the start-up switch over.*/

static volatile uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = 0;
static volatile uint16_t hStopPermanencyCounterM1 = 0;

static uint8_t UDC_Channel = 0u;
static uint16_t UDC_ConvertedValue = 0u;
static volatile UDRC_State_t UDC_State = UDRC_STATE_IDLE;

static bool SWO_transitionStartM1 = false;
uint8_t bMCBootCompleted = 0;
//}}}

//{{{
/**
	* @brief  It re-initializes the current and voltage variables. Moreover
	*         it clears qd currents PI controllers, voltage sensor and SpeednTorque
	*         controller. It must be called before each motor restart.
	*         It does not clear speed sensor.
	* @param  bMotor related motor it can be M1 or M2
	* @retval none
	*/
static void FOC_Clear (uint8_t bMotor)
{
	Curr_Components Inull = {(int16_t)0, (int16_t)0};
	Volt_Components Vnull = {(int16_t)0, (int16_t)0};

	FOCVars[bMotor].Iab = Inull;
	FOCVars[bMotor].Ialphabeta = Inull;
	FOCVars[bMotor].Iqd = Inull;
	FOCVars[bMotor].Iqdref = Inull;
	FOCVars[bMotor].hTeref = (int16_t)0;
	FOCVars[bMotor].Vqd = Vnull;
	FOCVars[bMotor].Valphabeta = Vnull;
	FOCVars[bMotor].hElAngle = (int16_t)0;

	PID_SetIntegralTerm (pPIDIq[bMotor], (int32_t)0);
	PID_SetIntegralTerm (pPIDId[bMotor], (int32_t)0);

	STC_Clear(pSTC[bMotor]);

	PWMC_SwitchOffPWM (pwmcHandle[bMotor]);
	}
//}}}
//{{{
static void FOC_InitAdditionalMethods (uint8_t bMotor) {
	}
//}}}
//{{{
static void FOC_CalcCurrRef (uint8_t bMotor) {

	if (FOCVars[bMotor].bDriveInput == INTERNAL) {
		FOCVars[bMotor].hTeref = STC_CalcTorqueReference (pSTC[bMotor]);
		FOCVars[bMotor].Iqdref.qI_Component1 = FOCVars[bMotor].hTeref;
		}
	}
//}}}

//{{{
static void TSK_SetChargeBootCapDelayM1 (uint16_t hTickCount)
{
	 hBootCapDelayCounterM1 = hTickCount;
}
//}}}
//{{{
static bool TSK_ChargeBootCapDelayHasElapsedM1()
{
	bool retVal = false;
	if (hBootCapDelayCounterM1 == 0)
		retVal = true;
	return (retVal);
}
//}}}
//{{{
static void TSK_SetStopPermanencyTimeM1 (uint16_t hTickCount)
{
	hStopPermanencyCounterM1 = hTickCount;
}
//}}}
//{{{
static bool TSK_StopPermanencyTimeHasElapsedM1()
{
	bool retVal = false;
	if (hStopPermanencyCounterM1 == 0)
		retVal = true;
	return (retVal);
}
//}}}

//{{{
static void TSK_MediumFrequencyTask() {

	int16_t wAux = 0;
	bool IsSpeedReliable = STO_PLL_CalcAvrgMecSpeed01Hz (&STO_PLL_M1, &wAux);
	PQD_CalcElMotorPower (pMPM[M1]);

	State_t StateM1 = STM_GetState(&STM[M1]);
	switch(StateM1) {
		//{{{
		case IDLE_START:
			R3_4_F30X_TurnOnLowSides(pwmcHandle[M1]);
			TSK_SetChargeBootCapDelayM1(CHARGE_BOOT_CAP_TICKS);
			STM_NextState(&STM[M1],CHARGE_BOOT_CAP);

			printf ("IDLE_START\n");
			break;
		//}}}
		//{{{
		case CHARGE_BOOT_CAP:
			if (TSK_ChargeBootCapDelayHasElapsedM1())
			{
				PWMC_CurrentReadingCalibr(pwmcHandle[M1],CRC_START);
				/* USER CODE BEGIN MediumFrequencyTask M1 Charge BootCap elapsed */

				/* USER CODE END MediumFrequencyTask M1 Charge BootCap elapsed */
				STM_NextState(&STM[M1],OFFSET_CALIB);
			}
			printf ("CHARGE_BOOT_CAP\n");
			break;
		//}}}
		//{{{
		case OFFSET_CALIB:
			if (PWMC_CurrentReadingCalibr(pwmcHandle[M1],CRC_EXEC))
				STM_NextState(&STM[M1],CLEAR);
			printf ("OFFSET_CALIB\n");
			break;
		//}}}
		//{{{
		case CLEAR:
			FOCVars[M1].bDriveInput = EXTERNAL;                               /* only for sensorless */
			STC_SetSpeedSensor(pSTC[M1],&VirtualSpeedSensorM1._Super);                             /* only for sensorless */
			RUC_Clear(&RevUpControlM1,MCI_GetImposedMotorDirection(oMCInterface[M1]));/* only for sensorless */
			SWO_transitionStartM1 = false;                                    /* only for sensorless */
			STO_PLL_Clear(&STO_PLL_M1);
			if (STM_NextState(&STM[M1], START) == true) {
				FOC_Clear(M1);
				R3_4_F30X_SwitchOnPWM(pwmcHandle[M1]);
				}

			printf ("CLEAR\n");
			break;
		//}}}
		//{{{
		case START: {
			/* only for sensor-less control */
			Curr_Components IqdRef;
			if (!RUC_Exec (&RevUpControlM1))
				/*Time allowed for startup has ended*/
				STM_FaultProcessing (&STM[M1], MC_START_UP, 0);
			else if (SWO_transitionStartM1 == false) {
				IqdRef.qI_Component1 = STC_CalcTorqueReference (pSTC[M1]);
				IqdRef.qI_Component2 = FOCVars[M1].UserIdref;
				FOCVars[M1].Iqdref = IqdRef;
				}

			int16_t hForcedMecSpeed01Hz;
			bool StartUpTransitionEnded = VSS_CalcAvrgMecSpeed01Hz (&VirtualSpeedSensorM1,&hForcedMecSpeed01Hz);
			bool StartUpDoTransition = VSS_SetStartTransition (&VirtualSpeedSensorM1, STO_PLL_IsObserverConverged (&STO_PLL_M1, hForcedMecSpeed01Hz));
			if (VSS_IsTransitionOngoing (&VirtualSpeedSensorM1)) {
				if (SWO_transitionStartM1 == false) {
					int16_t Iq = 0;
					Curr_Components StatorCurrent = MCM_Park (FOCVars[M1].Ialphabeta, SPD_GetElAngle (&STO_PLL_M1._Super));
					Iq = StatorCurrent.qI_Component1;

					REMNG_Init (pREMNG[M1]);
					REMNG_ExecRamp (pREMNG[M1], FOCVars[M1].Iqdref.qI_Component1, 0);
					REMNG_ExecRamp (pREMNG[M1], Iq, TRANSITION_DURATION);
					SWO_transitionStartM1 = true;
					}
				}
			else if (SWO_transitionStartM1 == true)
				SWO_transitionStartM1 = false;

			if (StartUpDoTransition == false)
				StartUpTransitionEnded = true;

			if (StartUpTransitionEnded == true) {
				#if (PID_SPEED_INTEGRAL_INIT_DIV == 0)
					PID_SetIntegralTerm (pPIDSpeed[M1], 0);
				#else
					PID_SetIntegralTerm (pPIDSpeed[M1],(int32_t)(FOCVars[M1].Iqdref.qI_Component1*PID_GetKIDivisor(pPIDSpeed[M1])/PID_SPEED_INTEGRAL_INIT_DIV));
				#endif

				printf ("STARTed\n");
				STM_NextState (&STM[M1], START_RUN);
				}
			}

			break;
		//}}}
		//{{{
		case START_RUN:
			/* only for sensor-less control */
			STC_SetSpeedSensor (pSTC[M1], &STO_PLL_M1._Super); /*Observer has converged*/

			{
			FOC_InitAdditionalMethods (M1);
			FOC_CalcCurrRef (M1);
			STM_NextState (&STM[M1], RUN);
			}

			STC_ForceSpeedReferenceToCurrentSpeed (pSTC[M1]); /* Init the reference speed to current speed */
			MCI_ExecBufferedCommands (oMCInterface[M1]); /* Exec the speed ramp after changing of the speed sensor */

			//printf ("IDLE_START\n");
			break;
		//}}}
		//{{{
		case RUN:
			MCI_ExecBufferedCommands (oMCInterface[M1]);
			FOC_CalcCurrRef (M1);

			if (!IsSpeedReliable) {
				printf ("TSK_MediumFrequencyTaskM1 - unreliable speed ignored\n");
				//STM_FaultProcessing (&STM[M1], MC_SPEED_FDBK, 0);
				}
			//printf ("START_RUN\n");
			break;
		//}}}
		//{{{
		case ANY_STOP:

			R3_4_F30X_SwitchOffPWM (pwmcHandle[M1]);
			FOC_Clear (M1);
			MPM_Clear ((MotorPowMeas_Handle_t*)pMPM[M1]);
			TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
			STM_NextState (&STM[M1], STOP);

			printf ("ANY_STOP\n");
			break;
		//}}}
		//{{{
		case STOP:

			if (TSK_StopPermanencyTimeHasElapsedM1()) {
				printf ("STOPed\n");
				STM_NextState (&STM[M1], STOP_IDLE);
				}

			break;
		//}}}
		//{{{
		case STOP_IDLE:

			STC_SetSpeedSensor (pSTC[M1],&VirtualSpeedSensorM1._Super);    /*  sensor-less */
			VSS_Clear (&VirtualSpeedSensorM1); /* Reset measured speed in IDLE */
			STM_NextState (&STM[M1], IDLE);

			printf ("STOP_IDLEd\n");
			break;
		//}}}
		default:
			break;
		}
	}
//}}}
//{{{
static void TSK_SafetyTask_PWMOFF (uint8_t bMotor) {

	uint16_t CodeReturn = NTC_CalcAvTemp (pTemperatureSensor[bMotor]); /* Clock temperature sensor and check for fault. It returns MC_OVER_TEMP or MC_NO_ERROR */
	CodeReturn |= PWMC_CheckOverCurrent (pwmcHandle[bMotor]); /* Clock current sensor and check for fault. It return MC_BREAK_IN or MC_NO_FAULTS (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
	if (bMotor == M1)
		CodeReturn |= RVBS_CalcAvVbusFilt (pBusSensorM1);

	/* Update the STM according error code */
	STM_FaultProcessing (&STM[bMotor], CodeReturn, ~CodeReturn);

	/* Acts on PWM outputs in case of faults */
	switch (STM_GetState (&STM[bMotor])) {
		case FAULT_NOW:
			PWMC_SwitchOffPWM (pwmcHandle[bMotor]);
			FOC_Clear (bMotor);
			MPM_Clear ((MotorPowMeas_Handle_t*)pMPM[bMotor]);
			break;

		case FAULT_OVER:
			PWMC_SwitchOffPWM (pwmcHandle[bMotor]);
			break;

		default:
			break;
		}
	}
//}}}

//{{{
static uint16_t FOC_CurrController (uint8_t bMotor) {
/**
	* @brief It executes the core of FOC drive that is the controllers for Iqd
	*        currents regulation. Reference frame transformations are carried out
	*        accordingly to the active speed sensor. It must be called periodically
	*        when new motor currents have been converted
	* @param this related object of class CFOC.
	* @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
	*         next PWM Update event, MC_FOC_DURATION otherwise
	*/
	Curr_Components Iab, Ialphabeta, Iqd;
	Volt_Components Valphabeta, Vqd;
	int16_t hElAngledpp;
	uint16_t hCodeError;

	hElAngledpp = SPD_GetElAngle(STC_GetSpeedSensor(pSTC[bMotor]));
	PWMC_GetPhaseCurrents(pwmcHandle[bMotor], &Iab);
	Ialphabeta = MCM_Clarke(Iab);

	Iqd = MCM_Park(Ialphabeta, hElAngledpp);
	Vqd.qV_Component1 = PI_Controller (pPIDIq[bMotor], (int32_t)(FOCVars[bMotor].Iqdref.qI_Component1) - Iqd.qI_Component1);
	Vqd.qV_Component2 = PI_Controller (pPIDId[bMotor], (int32_t)(FOCVars[bMotor].Iqdref.qI_Component2) - Iqd.qI_Component2);
	FOCVars[bMotor].Vqd = Vqd;
	Vqd = Circle_Limitation (pCLM[bMotor], Vqd);

	Valphabeta = MCM_Rev_Park (Vqd, hElAngledpp);

	hCodeError = PWMC_SetPhaseVoltage (pwmcHandle[bMotor], Valphabeta);
	FOCVars[bMotor].Iab = Iab;
	FOCVars[bMotor].Ialphabeta = Ialphabeta;
	FOCVars[bMotor].Iqd = Iqd;
	FOCVars[bMotor].Valphabeta = Valphabeta;
	FOCVars[bMotor].hElAngle = hElAngledpp;

	return hCodeError;
	}
//}}}

// interface
//{{{
/**
* @brief  This function requests a user-defined regular conversion. All user
*         defined conversion requests must be performed inside routines with the
*         same priority level. If previous regular conversion request is pending
*         this function has no effect, for this reason is better to call the
*         MC_RegularConvState and check if the state is UDRC_STATE_IDLE before
*         to call MC_RequestRegularConv.
* @param  bChannel ADC channel used for the regular conversion.
* @param  bSamplTime Sampling time selection, ADC_SampleTime_nCycles defined in
*         stm32fxxx_adc.h see ADC_sampling_times.
*/
void MC_RequestRegularConv (uint8_t bChannel, uint8_t bSamplTime) {

	ADConv_t ADConv_struct;
	if (UDC_State == UDRC_STATE_IDLE) {
		ADConv_struct.Channel = bChannel;
		ADConv_struct.SamplTime = bSamplTime;
		PWMC_ADC_SetSamplingTime(pwmcHandle[M1],ADConv_struct);
		UDC_State = UDRC_STATE_REQUESTED;
		UDC_Channel = bChannel;
		}
	}
//}}}
//{{{
uint16_t MC_GetRegularConv() {

	uint16_t hRetVal = 0xFFFFu;
	if (UDC_State == UDRC_STATE_EOC) {
		hRetVal = UDC_ConvertedValue;
		UDC_State = UDRC_STATE_IDLE;
		}

	return hRetVal;
	}
//}}}
//{{{
UDRC_State_t MC_RegularConvState() {
	return UDC_State;
	}
//}}}

//{{{
MCI_Handle_t* GetMCI (uint8_t bMotor) {

	MCI_Handle_t* retVal = MC_NULL;
	if ((oMCInterface != MC_NULL) && (bMotor < MC_NUM))
		retVal = oMCInterface[bMotor];
	return retVal;
	}
//}}}
//{{{
MCT_Handle_t* GetMCT (uint8_t bMotor) {

	MCT_Handle_t* retVal = MC_NULL;
	if (bMotor < MC_NUM)
		retVal = &MCT[bMotor];
	return retVal;
	}
//}}}

//{{{
uint8_t TSK_HighFrequencyTask() {

	Observer_Inputs_t STO_Inputs;
	STO_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;
	if (SWO_transitionStartM1 == true)
		if (!REMNG_RampCompleted (pREMNG[M1]))
			FOCVars[M1].Iqdref.qI_Component1 = REMNG_Calc (pREMNG[M1]);

	uint16_t hFOCreturn = FOC_CurrController (M1);
	if (hFOCreturn == MC_FOC_DURATION) {
		//STM_FaultProcessing (&STM[M1], MC_FOC_DURATION, 0);
		printf ("TSK_HighFrequencyTask - MC_FOC_DURATION ignored\n");
		}
	else {
		bool IsAccelerationStageReached = RUC_FirstAccelerationStageReached (&RevUpControlM1);
		STO_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta;
		STO_Inputs.Vbus = VBS_GetAvBusVoltage_d (&(pBusSensorM1->_Super));
		STO_PLL_CalcElAngle (&STO_PLL_M1, &STO_Inputs);
		STO_PLL_CalcAvrgElSpeedDpp (&STO_PLL_M1);

	 if (IsAccelerationStageReached == false)
			STO_ResetPLL (&STO_PLL_M1);

		uint16_t hState = STM_GetState (&STM[M1]);
		if((hState == START) || (hState == START_RUN)) {
			/*  only for sensor-less*/
			int16_t hObsAngle = SPD_GetElAngle(&STO_PLL_M1._Super);
			VSS_CalcElAngle(&VirtualSpeedSensorM1,&hObsAngle);
			}
		}

	return 0;
	}
//}}}
//{{{
void TSK_SafetyTask() {

	if (bMCBootCompleted == 1) {
		TSK_SafetyTask_PWMOFF (M1);

		if (UDC_State == UDRC_STATE_REQUESTED) {
			UDC_ConvertedValue = PWMC_ExecRegularConv (pwmcHandle[M1], UDC_Channel);
			UDC_State = UDRC_STATE_EOC;
			}
		}
	}
//}}}
//{{{
void TSK_HardwareFaultTask() {

	R3_4_F30X_SwitchOffPWM (pwmcHandle[M1]);
	STM_FaultProcessing (&STM[M1], MC_SW_ERROR, 0);
	}
//}}}

//{{{
void MC_Scheduler() {

	if (bMCBootCompleted == 1) {

		if(hMFTaskCounterM1 > 0u)
			hMFTaskCounterM1--;
		else {
			TSK_MediumFrequencyTask();
			hMFTaskCounterM1 = MF_TASK_OCCURENCE_TICKS;
			}

		if (hBootCapDelayCounterM1 > 0u)
			hBootCapDelayCounterM1--;
		if (hStopPermanencyCounterM1 > 0u)
			hStopPermanencyCounterM1--;
		}
	}
//}}}
//{{{
void MCboot (MCI_Handle_t* pMCIList[NBR_OF_MOTORS],MCT_Handle_t* pMCTList[NBR_OF_MOTORS] )
{
	/* USER CODE BEGIN MCboot 0 */

	/* USER CODE END MCboot 0 */
	bMCBootCompleted = 0;
	pCLM[M1] = &CircleLimitationM1;

	/**********************************************************/
	/*    PWM and current sensing component initialization    */
	/**********************************************************/
	pwmcHandle[M1] = &PWMC_R3_4_F3_Handle_M1._Super;
	R3_4_F30X_Init(&PWMC_R3_4_F3_Handle_M1);

	/* USER CODE BEGIN MCboot 1 */

	/* USER CODE END MCboot 1 */

	/**************************************/
	/*    Start timers synchronously      */
	/**************************************/
	startTimers();

	/**************************************/
	/*    State machine initialization    */
	/**************************************/
	STM_Init(&STM[M1]);

	/******************************************************/
	/*   PID component initialization: speed regulation   */
	/******************************************************/
	PID_HandleInit(&PIDSpeedHandle_M1);
	pPIDSpeed[M1] = &PIDSpeedHandle_M1;
	pSTC[M1] = &SpeednTorqCtrlM1;
	STO_PLL_Init (&STO_PLL_M1);

	STC_Init(pSTC[M1],pPIDSpeed[M1], &STO_PLL_M1._Super);
	VSS_Init (&VirtualSpeedSensorM1);
	RUC_Init(&RevUpControlM1,pSTC[M1],&VirtualSpeedSensorM1, &STO_M1, pwmcHandle[M1]);        /* only if sensorless*/

	/********************************************************/
	/*   PID component initialization: current regulation   */
	/********************************************************/
	PID_HandleInit(&PIDIqHandle_M1);
	PID_HandleInit(&PIDIdHandle_M1);
	pPIDIq[M1] = &PIDIqHandle_M1;
	pPIDId[M1] = &PIDIdHandle_M1;
	pBusSensorM1 = &RealBusVoltageSensorParamsM1;
	RVBS_Init(pBusSensorM1, pwmcHandle[M1]);

	//Power Measurement M1
	pMPM[M1] = &PQD_MotorPowMeasM1;
	pMPM[M1]->pVBS = &(pBusSensorM1->_Super);
	pMPM[M1]->pFOCVars = &FOCVars[M1];

	NTC_Init(&TempSensorParamsM1,pwmcHandle[M1]);
	pTemperatureSensor[M1] = &TempSensorParamsM1;

	pREMNG[M1] = &RampExtMngrHFParamsM1;
	REMNG_Init(pREMNG[M1]);
	FOC_Clear(M1);
	FOCVars[M1].bDriveInput = EXTERNAL;
	FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
	FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).qI_Component2;
	oMCInterface[M1] = & Mci[M1];
	MCI_Init(oMCInterface[M1], &STM[M1], pSTC[M1], &FOCVars[M1]);
	MCI_ExecSpeedRamp(oMCInterface[M1],
	STC_GetMecSpeedRef01HzDefault(pSTC[M1]),0); /*First command to STC*/
	pMCIList[M1] = oMCInterface[M1];
	MCT[M1].pPIDSpeed = pPIDSpeed[M1];
	MCT[M1].pPIDIq = pPIDIq[M1];
	MCT[M1].pPIDId = pPIDId[M1];
	MCT[M1].pPIDFluxWeakening = MC_NULL; /* if M1 doesn't has FW */
	MCT[M1].pPWMnCurrFdbk = pwmcHandle[M1];
	MCT[M1].pRevupCtrl = &RevUpControlM1;              /* only if M1 is sensorless*/
	MCT[M1].pSpeedSensorMain = (SpeednPosFdbk_Handle_t *) &STO_PLL_M1;
	MCT[M1].pSpeedSensorAux = MC_NULL;
	MCT[M1].pSpeedSensorVirtual = &VirtualSpeedSensorM1;  /* only if M1 is sensorless*/
	MCT[M1].pSpeednTorqueCtrl = pSTC[M1];
	MCT[M1].pStateMachine = &STM[M1];
	MCT[M1].pTemperatureSensor = (NTC_Handle_t *) pTemperatureSensor[M1];
	MCT[M1].pBusVoltageSensor = &(pBusSensorM1->_Super);
	MCT[M1].pBrakeDigitalOutput = MC_NULL;   /* brake is defined, oBrakeM1*/
	MCT[M1].pNTCRelay = MC_NULL;             /* relay is defined, oRelayM1*/
	MCT[M1].pMPM =  (MotorPowMeas_Handle_t*)pMPM[M1];
	MCT[M1].pFW = MC_NULL;
	MCT[M1].pFF = MC_NULL;
	MCT[M1].pSCC = MC_NULL;
	MCT[M1].pOTT = MC_NULL;
	pMCTList[M1] = &MCT[M1];

	bMCBootCompleted = 1;
	/* USER CODE BEGIN MCboot 2 */

	/* USER CODE END MCboot 2 */
}
//}}}

//{{{
void mc_lock_pins() {

	HAL_GPIO_LockPin(M1_CURR_AMPL_W_GPIO_Port, M1_CURR_AMPL_W_Pin);
	HAL_GPIO_LockPin(M1_PWM_UH_GPIO_Port, M1_PWM_UH_Pin);
	HAL_GPIO_LockPin(M1_PWM_VH_GPIO_Port, M1_PWM_VH_Pin);
	HAL_GPIO_LockPin(M1_OCP_GPIO_Port, M1_OCP_Pin);
	HAL_GPIO_LockPin(M1_PWM_WH_GPIO_Port, M1_PWM_WH_Pin);
	HAL_GPIO_LockPin(M1_PWM_EN_W_GPIO_Port, M1_PWM_EN_W_Pin);
	HAL_GPIO_LockPin(M1_PWM_EN_V_GPIO_Port, M1_PWM_EN_V_Pin);
	HAL_GPIO_LockPin(M1_PWM_EN_U_GPIO_Port, M1_PWM_EN_U_Pin);
	HAL_GPIO_LockPin(M1_BUS_VOLTAGE_GPIO_Port, M1_BUS_VOLTAGE_Pin);
	HAL_GPIO_LockPin(M1_CURR_AMPL_U_GPIO_Port, M1_CURR_AMPL_U_Pin);
	HAL_GPIO_LockPin(M1_CURR_AMPL_V_GPIO_Port, M1_CURR_AMPL_V_Pin);
	HAL_GPIO_LockPin(M1_TEMPERATURE_GPIO_Port, M1_TEMPERATURE_Pin);
	}
//}}}
