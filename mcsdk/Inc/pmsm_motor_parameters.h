#ifndef __PMSM_MOTOR_PARAMETERS_H
#define __PMSM_MOTOR_PARAMETERS_H

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM          4 /* Number of motor pole pairs */
#define RS                     1.10 /* Stator resistance , ohm*/
#define LS                     0.000147 /* Stator inductance, H
                                                 For I-PMSM it is equal to Lq */

/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the
   PID for speed regulation (i.e. reference torque).
   Transformation of real currents (A) into int16_t format must be done accordingly with formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#define NOMINAL_CURRENT         20053
#define MOTOR_MAX_SPEED_RPM     7000 /*!< Maximum rated speed  */
#define MOTOR_VOLTAGE_CONSTANT  2.4 /*!< Volts RMS ph-ph /kRPM */
#define ID_DEMAG                -20053 /*!< Demagnetization current */

/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_PLACEMENT  DEGREES_120 /*!<Define here the
                                                 mechanical position of the sensors
                                                 withreference to an electrical cycle.
                                                 It can be either DEGREES_120 or
                                                 DEGREES_60 */

#define HALL_PHASE_SHIFT        300 /*!< Define here in degrees
                                                 the electrical phase shift between
                                                 the low to high transition of
                                                 signal H1 and the maximum of
                                                 the Bemf induced on phase A */
/*** Quadrature encoder ***/
#define ENCODER_PPR             400  /*!< Number of pulses per
                                            revolution */

#endif /*__PMSM_MOTOR_PARAMETERS_H*/
