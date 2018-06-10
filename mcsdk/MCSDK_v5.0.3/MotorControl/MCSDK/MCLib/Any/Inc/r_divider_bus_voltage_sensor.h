#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//}}}

#include "pwm_curr_fdbk.h"
#include "bus_voltage_sensor.h"

#define BUS_BUFF_MAX                10   /* Maximum buffer size to compute average value.*/

/**
  * @brief  Rdivider class parameters definition
  */
typedef struct
{
  BusVoltageSensor_Handle_t _Super;

  uint8_t        VbusADChannel;         /*!< ADC channel used for conversion of
                                             bus voltage. It must be equal to
                                             ADC_CHANNEL_xx x= 0, ..., 15*/
  GPIO_TypeDef*  VbusPort;              /*!< GPIO port used by bVbusADChannel.
                                             It must be equal to GPIOx x= A, B, ...*/
  uint16_t       VbusPin;               /*!< GPIO pin used by bVbusChannel. It must
                                             be equal to GPIO_Pin_x x= 0, 1, ...*/
  uint8_t        VbusSamplingTime;      /*!< Sampling time used for bVbusChannel AD
                                             conversion. It must be equal to
                                             ADC_SampleTime_xCycles5 x= 1, 7, ...*/
  uint16_t       LowPassFilterBW;       /*!< Use this number to configure the Vbus
                                             first order software filter bandwidth.
                                             hLowPassFilterBW = VBS_CalcBusReading
                                             call rate [Hz]/ FilterBandwidth[Hz] */
  uint16_t       OverVoltageThreshold;  /*!< It represents the over voltage protection
                                             intervention threshold. To be expressed
                                             in digital value through formula:
                                             hOverVoltageThreshold (digital value) =
                                             Over Voltage Threshold (V) * 65536
                                             / hConversionFactor */
  uint16_t       UnderVoltageThreshold; /*!< It represents the under voltage protection
                                             intervention threshold. To be expressed
                                             in digital value through formula:
                                             hUnderVoltageThreshold (digital value)=
                                             Under Voltage Threshold (V) * 65536
                                             / hConversionFactor */
  PWMC_Handle_t* PWMnCurrentSensor;     /*!< PWMC_Handle_t to be used for regular
                                             conversions*/
  uint16_t       aBuffer[BUS_BUFF_MAX]; /*!< Buffer used to compute average value.*/
  uint8_t        elem;                  /*!< Number of stored elements in the average buffer.*/
  uint8_t        index;                 /*!< Index of last stored element in the average buffer.*/

}RDivider_Handle_t;

void RVBS_Init(RDivider_Handle_t *pHandle, PWMC_Handle_t* PWMnCurrentSensor);
void RVBS_Clear(RDivider_Handle_t *pHandle);
uint16_t RVBS_CalcAvVbusFilt(RDivider_Handle_t *pHandle);
uint16_t RVBS_CalcAvVbus(RDivider_Handle_t *pHandle);
uint16_t RVBS_CheckFaultState(RDivider_Handle_t *pHandle);

//{{{
#ifdef __cplusplus
}
#endif /* __cpluplus */
//}}}
