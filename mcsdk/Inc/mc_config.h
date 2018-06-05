#pragma once

#include "pid_regulator.h"
#include "revup_ctrl.h"
#include "speed_torq_ctrl.h"
#include "virtual_speed_sensor.h"
#include "ntc_temperature_sensor.h"
#include "pwm_curr_fdbk.h"
#include "r_divider_bus_voltage_sensor.h"
#include "virtual_bus_voltage_sensor.h"
#include "pqd_motor_power_measurement.h"
#include "user_interface.h"
#include "lcd_manager_ui.h"
#include "lcd_vintage_ui.h"
#include "dac_rctimer_ui.h"
#include "motor_control_protocol.h"
#include "r3_4_f30x_pwm_curr_fdbk.h"
extern RevUpCtrl_Handle_t RevUpControlM1;
#include "ramp_ext_mngr.h"
#include "circle_limitation.h"
#include "sto_speed_pos_fdbk.h"
#include "sto_pll_speed_pos_fdbk.h"
#include "usart_frame_communication_protocol.h"

extern PID_Handle_t PIDSpeedHandle_M1;
extern PID_Handle_t PIDIqHandle_M1;
extern PID_Handle_t PIDIdHandle_M1;
extern NTC_Handle_t TempSensorParamsM1;
extern PWMC_R3_4_F3_Handle_t PWMC_R3_4_F3_Handle_M1;
extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1;
extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1;
extern PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1;
extern VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1;
extern VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1;
extern STO_Handle_t STO_M1;
extern STO_PLL_Handle_t STO_PLL_M1;
extern NTC_Handle_t TempSensorParamsM1;
extern RDivider_Handle_t RealBusVoltageSensorParamsM1;
extern CircleLimitation_Handle_t CircleLimitationM1;
extern UI_Handle_t UI_Params;
extern RampExtMngr_Handle_t RampExtMngrHFParamsM1;
extern UFCP_Handle_t pUSART;
