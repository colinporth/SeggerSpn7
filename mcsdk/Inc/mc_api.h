#pragma once
#include "mc_type.h"
#include "mc_interface.h"
#include "state_machine.h"
//{{{
#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//}}}

typedef enum {UDRC_STATE_IDLE, UDRC_STATE_REQUESTED, UDRC_STATE_EOC} UDRC_State_t;

bool MC_StartMotor1();
void MC_StopMotor1();

void MC_ProgramSpeedRampMotor1( int16_t hFinalSpeed, uint16_t hDurationms );
void MC_ProgramTorqueRampMotor1( int16_t hFinalTorque, uint16_t hDurationms );
void MC_SetCurrentReferenceMotor1( Curr_Components Iqdref );

MCI_CommandState_t MC_GetCommandStateMotor1( void);

/* Stops the execution of the current speed ramp for Motor 1 if any */
bool MC_StopSpeedRampMotor1();

/* Returns true if the last submited ramp for Motor 1 has completed, false otherwise */
bool MC_HasRampCompletedMotor1();

/* Returns the current mechanical rotor speed reference set for Motor 1, expressed in dHz (tenth of Hertz) */
int16_t MC_GetMecSpeedReferenceMotor1();

/* Returns the last computed average mechanical rotor speed for Motor 1, expressed in dHz (tenth of Hertz) */
int16_t MC_GetMecSpeedAverageMotor1();

/* Returns the final speed of the last ramp programmed for Motor 1, if this ramp was a speed ramp */
int16_t MC_GetLastRampFinalSpeedMotor1();

/* Returns the current Control Mode for Motor 1 (either Speed or Torque) */
STC_Modality_t MC_GetControlModeMotor1();

/* Returns the direction imposed by the last command on Motor 1 */
int16_t MC_GetImposedDirectionMotor1(void);

/* Returns the current reliability of the speed sensor used for Motor 1 */
bool MC_GetSpeedSensorReliabilityMotor1();

/* returns the amplitude of the phase current injected in Motor 1 */
int16_t MC_GetPhaseCurrentAmplitudeMotor1();

/* returns the amplitude of the phase voltage applied to Motor 1 */
int16_t MC_GetPhaseVoltageAmplitudeMotor1();

/* returns current Ia and Ib values for Motor 1 */
Curr_Components MC_GetIabMotor1();

/* returns current Ialpha and Ibeta values for Motor 1 */
Curr_Components MC_GetIalphabetaMotor1();

/* returns current Iq and Id values for Motor 1 */
Curr_Components MC_GetIqdMotor1();

/* returns Iq and Id reference values for Motor 1 */
Curr_Components MC_GetIqdrefMotor1();

/* returns current Vq and Vd values for Motor 1 */
Volt_Components MC_GetVqdMotor1();

/* returns current Valpha and Vbeta values for Motor 1 */
Volt_Components MC_GetValphabetaMotor1(void);

/* returns the electrical angle of the rotor of Motor 1, in DDP format */
int16_t MC_GetElAngledppMotor1();

/* returns the current electrical torque reference for Motor 1 */
int16_t MC_GetTerefMotor1();

/* Sets the reference value for Id */
void MC_SetIdrefMotor1 (int16_t hNewIdref);

/* re-initializes Iq and Id references to their default values */
void MC_Clear_IqdrefMotor1();

/* Acknowledge a Motor Control fault on Motor 1 */
bool MC_AcknowledgeFaultMotor1();

/* Returns a bitfiled showing faults that occured since the State Machine of Motor 1 was moved to FAULT_NOW state */
uint16_t MC_GetOccurredFaultsMotor1();

/* Returns a bitfield showing all current faults on Motor 1 */
uint16_t MC_GetCurrentFaultsMotor1();

/* returns the current state of Motor 1 state machine */
State_t MCI_GetSTMStateMotor1();

/* programs a user defined ADC regular conversion */
void MC_ProgramRegularConversion (uint8_t bChannel, uint8_t bSampleTime);

/* Returns the value of the last executed user defined ADC regular conversion */
uint16_t MC_GetRegularConversionValue();

/* Returns the status of the last requested user defined ADC regular conversion */
UDRC_State_t MC_GetRegularConversionState();

//{{{
#ifdef __cplusplus
}
#endif /* __cpluplus */
//}}}
