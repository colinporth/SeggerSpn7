#pragma once

#define NUM_POLE_PAIRS                       4    // Number of Motor Pole pairs
#define DIRECTION                            0    // Set motor direction CW = 0 and CCW = 1
#define TARGET_SPEED                      2000    // Target speed in closed loop control

#define STARTUP_CURRENT_REFERENCE         1500    // StartUP Currente Reference (2000 = 2.2A)
#define ACC                             600000    // Mechanical acceleration rate (setting available in manual mode, LOAD_TYPE = 0)
#define MINIMUM_ACC                        500    // Mechanical acceleration rate for BIG load application
#define NUMBER_OF_STEPS                  20000    // Number of elements for motor start-UP (max value 65535)
#define TIME_FOR_ALIGN                     500    // Time for alignment (msec)
#define BUTTON_DELAY                       200    // Delay time to enable push button for new command (1 = 1msec)
#define NUMBER_ZCR                          12    // Number of zero crossing event during the startup for closed loop control begin

#define SPEED_LOOP_TIME                      1    // Speed Loop time (1 = 1msec)
#define KP_GAIN                           8000    // Kp parameter for PI regulator
#define KI_GAIN                             50    // Ki parameter for PI regulator
#define KP_DIV                            4096    // Kp parameter divider for PI regulator
#define KI_DIV                            4096    // Ki parameter divider for PI regulator
#define LOWER_OUT_LIMIT                    120    // Low Out value of PI regulator
#define UPPER_OUT_LIMIT                   2000    // High Out value of PI regulator
#define MAX_POT_SPEED                     5000    // Maximum Speed regulated by potentiometer
#define MIN_POT_SPEED                     2000    // Minimum Speed regulated by potentiometer
#define VAL_POT_SPEED_DIV                    2    // Validation potentiometer speed divider
#define INITIAL_DEMAGN_DELAY                10    // Initial value for delay time during startup for Bemf detection

// Zero Crossissing parameters
#define BEMF_THRSLD_DOWN                    200   // Zero Crossing threshold
#define BEMF_THRSLD_UP                      200   // Zero Crossing threshold

// Speed filtering parameters
#define HFBUFFERSIZE                        10
#define FILTER_DEEP                         20    // Number of bits for digital filter
#define ADC_SPEED_TH                        82    // Fixed treshold to change the target speed (t.b.f)

// Motor stall detection parameters
#define BEMF_CONSEC_DOWN_MAX                10   // Maximum value of BEMF Consecutive Threshold Falling Crossings Counter in closed loop
#define BEMF_CNT_EVENT_MAX                 100   // Maximum number of BEMF Counter in open loop

// Look UP table for dynamic demagn control of speed
#define DEMAGN_VAL_1                         1   // Look UP table for dynamic demagn control for speed into (10000,12000] or [-12000,-10000) range
#define DEMAGN_VAL_2                         2   // Look UP table for dynamic demagn control for speed into ( 7800,10000] or [-10000,- 7800) range
#define DEMAGN_VAL_3                         3   // Look UP table for dynamic demagn control for speed into ( 6400, 7800] or [- 7800,- 6400) range
#define DEMAGN_VAL_4                         4   // Look UP table for dynamic demagn control for speed into ( 5400, 6400] or [- 6400,- 5400) range
#define DEMAGN_VAL_5                         5   // Look UP table for dynamic demagn control for speed into ( 4650, 5400] or [- 5400,- 4650) range
#define DEMAGN_VAL_6                         6   // Look UP table for dynamic demagn control for speed into ( 4100, 4650] or [- 4650,- 4100) range
#define DEMAGN_VAL_7                         7   // Look UP table for dynamic demagn control for speed into ( 3650, 4100] or [- 4100,- 3650) range
#define DEMAGN_VAL_8                         8   // Look UP table for dynamic demagn control for speed into ( 3300, 3650] or [- 3650,- 3300) range
#define DEMAGN_VAL_9                         9   // Look UP table for dynamic demagn control for speed into ( 2600, 3300] or [- 3300,- 2600) range
#define DEMAGN_VAL_10                       10   // Look UP table for dynamic demagn control for speed into ( 1800, 2600] or [- 2600,- 1800) range
#define DEMAGN_VAL_11                       11   // Look UP table for dynamic demagn control for speed into ( 1500, 1800] or [- 1800,- 1500) range
#define DEMAGN_VAL_12                       12   // Look UP table for dynamic demagn control for speed into ( 1300, 1500] or [- 1500,- 1300) range
#define DEMAGN_VAL_13                       13   // Look UP table for dynamic demagn control for speed into ( 1000, 1300] or [- 1300,- 1000) range
#define DEMAGN_VAL_14                       14   // Look UP table for dynamic demagn control for speed into [  500, 1000] or [- 1000,-  500] range

#define TRUE                                 1   // Define TRUE */
#define FALSE                                0   // Define FALSE */
