#pragma once

#define PWBDID 2

/************* PWM Driving signals section **************/
#define PHASE_UH_POLARITY             H_ACTIVE_HIGH
#define PHASE_VH_POLARITY             H_ACTIVE_HIGH
#define PHASE_WH_POLARITY             H_ACTIVE_HIGH

#define PHASE_UL_POLARITY             L_ACTIVE_HIGH
#define PHASE_VL_POLARITY             L_ACTIVE_HIGH
#define PHASE_WL_POLARITY             L_ACTIVE_HIGH

#define HW_DEAD_TIME_NS               2000 /*!< Dead-time inserted by HW if low side signals are not used */
/********** Inrush current limiter signal section *******/
#define INRUSH_CURR_LIMITER_POLARITY  DOUT_ACTIVE_HIGH

/******* Dissipative brake driving signal section *******/
#define DISSIPATIVE_BRAKE_POLARITY    DOUT_ACTIVE_HIGH

/*********** Bus voltage sensing section ****************/
#define VBUS_PARTITIONING_FACTOR      0.0522 /*!< It expresses how much the Vbus is attenuated before being converted into digital value */
#define NOMINAL_BUS_VOLTAGE_V         11

/******** Current reading parameters section ******/
/*** Topology ***/
#define THREE_SHUNT_INDEPENDENT_RESOURCES

#define RSHUNT                        0.330

/*  ICSs gains in case of isolated current sensors, amplification gain for shunts based sensing */
#define AMPLIFICATION_GAIN            1.53

/*** Noise parameters ***/
#define TNOISE_NS                     700
#define TRISE_NS                      700

/*********** Over-current protection section ************/
#define OVERCURR_FEEDBACK_POLARITY       EMSTOP2_ACTIVE_LOW
#define OVERCURR_PROTECTION_HW_DISABLING DOUT_ACTIVE_HIGH

/************ Temperature sensing section ***************/
/* V[V]=V0+dV/dT[V/Celsius]*(T-T0)[Celsius]*/
#define V0_V  1.055 /*!< in Volts */
#define T0_C  25    /*!< in Celsius degrees */
#define dV_dT 0.023 /*!< V/Celsius degrees */
#define T_MAX 110   /*!< Sensor measured temperature at maximum power stage working temperature, Celsius degrees */
