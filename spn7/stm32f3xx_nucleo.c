// stm32f3xx_nucleo.c
#include "stm32f3xx_nucleo.h"

//{{{  leds
#define LED2_PIN                  GPIO_PIN_5
#define LED2_GPIO_PORT            GPIOA
#define LED2_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOA_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   do { if ((__INDEX__) == 0) LED2_GPIO_CLK_ENABLE();} while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  (((__INDEX__) == 0) ? LED2_GPIO_CLK_DISABLE() : 0)
//}}}
//{{{  buttons
#define USER_BUTTON_PIN                  GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT            GPIOC
#define USER_BUTTON_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOC_CLK_DISABLE()
#define USER_BUTTON_EXTI_IRQn            EXTI15_10_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    do { if ((__INDEX__) == 0) USER_BUTTON_GPIO_CLK_ENABLE();} while(0)
#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == 0) ? USER_BUTTON_GPIO_CLK_DISABLE() : 0)
//}}}

#if defined(HAL_SPI_MODULE_ENABLED)
  //{{{  spi defines
  #define NUCLEO_SPIx                               SPI1
  #define NUCLEO_SPIx_CLK_ENABLE()                  __HAL_RCC_SPI1_CLK_ENABLE()

  #define NUCLEO_SPIx_SCK_AF                        GPIO_AF5_SPI1
  #define NUCLEO_SPIx_SCK_GPIO_PORT                 GPIOA
  #define NUCLEO_SPIx_SCK_PIN                       GPIO_PIN_5
  #define NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
  #define NUCLEO_SPIx_SCK_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()

  #define NUCLEO_SPIx_MISO_MOSI_AF                  GPIO_AF5_SPI1
  #define NUCLEO_SPIx_MISO_MOSI_GPIO_PORT           GPIOA
  #define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
  #define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOA_CLK_DISABLE()
  #define NUCLEO_SPIx_MISO_PIN                      GPIO_PIN_6
  #define NUCLEO_SPIx_MOSI_PIN                      GPIO_PIN_7

  #define NUCLEO_SPIx_TIMEOUT_MAX                   1000
  //}}}
#endif

#define SD_CS_LOW()    HAL_GPIO_WritePin (SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()   HAL_GPIO_WritePin (SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET)

#define SD_CS_PIN                  GPIO_PIN_5
#define SD_CS_GPIO_PORT            GPIOB
#define SD_CS_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define SD_CS_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()

#define LCD_CS_PIN                 GPIO_PIN_6
#define LCD_CS_GPIO_PORT           GPIOB
#define LCD_CS_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_CS_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOB_CLK_DISABLE()

#define LCD_DC_PIN                  GPIO_PIN_9
#define LCD_DC_GPIO_PORT            GPIOA
#define LCD_DC_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()
#define LCD_DC_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOA_CLK_DISABLE()


GPIO_TypeDef* LED_PORT[LEDn] = { LED2_GPIO_PORT };
const uint16_t LED_PIN[LEDn] = { LED2_PIN };

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = { USER_BUTTON_GPIO_PORT };
const uint16_t BUTTON_PIN[BUTTONn] = { USER_BUTTON_PIN };
const uint8_t BUTTON_IRQn[BUTTONn] = { USER_BUTTON_EXTI_IRQn };

//{{{
void BSP_LED_Init (Led_TypeDef Led) {

  LEDx_GPIO_CLK_ENABLE (Led);

  // Config GPIO_LED pin
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin = LED_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init (LED_PORT[Led], &GPIO_InitStruct);
  HAL_GPIO_WritePin (LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
  }
//}}}
//{{{
void BSP_LED_DeInit (Led_TypeDef Led) {

  // Turn off LED
  HAL_GPIO_WritePin (LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);

  // DeInit the GPIO_LED pin
  GPIO_InitTypeDef gpio_init_structure;
  gpio_init_structure.Pin = LED_PIN[Led];
  HAL_GPIO_DeInit (LED_PORT[Led], gpio_init_structure.Pin);
  }
//}}}
//{{{
void BSP_LED_On (Led_TypeDef Led) {
  HAL_GPIO_WritePin (LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET);
  }
//}}}
//{{{
void BSP_LED_Off (Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
}
//}}}
//{{{
void BSP_LED_Toggle (Led_TypeDef Led) {
  HAL_GPIO_TogglePin (LED_PORT[Led], LED_PIN[Led]);
  }
//}}}

//{{{
void BSP_PB_Init (Button_TypeDef Button, ButtonMode_TypeDef ButtonMode) {

  // Enable the BUTTON Clock
  BUTTONx_GPIO_CLK_ENABLE(Button);

  if (ButtonMode == BUTTON_MODE_GPIO) {
    // Configure Button pin as input
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init (BUTTON_PORT[Button], &GPIO_InitStruct);
    }

  if (ButtonMode == BUTTON_MODE_EXTI) {
    // Configure Button pin as input with External interrupt
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    HAL_GPIO_Init (BUTTON_PORT[Button], &GPIO_InitStruct);

    // Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority ((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0x00);
    HAL_NVIC_EnableIRQ ((IRQn_Type)(BUTTON_IRQn[Button]));
    }
  }
//}}}
//{{{
void BSP_PB_DeInit (Button_TypeDef Button) {

  GPIO_InitTypeDef gpio_init_structure;
  gpio_init_structure.Pin = BUTTON_PIN[Button];
  HAL_NVIC_DisableIRQ ((IRQn_Type)(BUTTON_IRQn[Button]));
  HAL_GPIO_DeInit (BUTTON_PORT[Button], gpio_init_structure.Pin);
  }
//}}}
//{{{
uint32_t BSP_PB_GetState (Button_TypeDef Button) {
  return HAL_GPIO_ReadPin (BUTTON_PORT[Button], BUTTON_PIN[Button]);
  }
//}}}

#ifdef HAL_SPI_MODULE_ENABLED
  //{{{  spi
  #define SD_DUMMY_BYTE            0xFF
  #define SD_NO_RESPONSE_EXPECTED  0x80

  uint32_t SpixTimeout = NUCLEO_SPIx_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */
  static SPI_HandleTypeDef hnucleo_Spi;

  static void       SPIx_Init();
  static void       SPIx_Write (uint8_t Value);
  static uint32_t   SPIx_Read();
  static void       SPIx_Error();
  static void       SPIx_MspInit (SPI_HandleTypeDef *hspi);

  /* SD IO functions */
  void              SD_IO_Init();
  HAL_StatusTypeDef SD_IO_WriteCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t Response);
  HAL_StatusTypeDef SD_IO_WaitResponse(uint8_t Response);
  void              SD_IO_WriteDummy();
  void              SD_IO_CSState(uint8_t state);
  void              SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength);
  uint8_t           SD_IO_WriteByte(uint8_t Data);
  uint8_t           SD_IO_ReadByte();

  //{{{
  static void SPIx_MspInit (SPI_HandleTypeDef *hspi)
  {
    GPIO_InitTypeDef  GPIO_InitStruct;

    /*** Configure the GPIOs ***/
    /* Enable GPIO clock */
    NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE();
    NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE();

    /* Configure SPI SCK */
    GPIO_InitStruct.Pin = NUCLEO_SPIx_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = NUCLEO_SPIx_SCK_AF;
    HAL_GPIO_Init(NUCLEO_SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

    /* Configure SPI MISO and MOSI */
    GPIO_InitStruct.Pin = NUCLEO_SPIx_MOSI_PIN;
    GPIO_InitStruct.Alternate = NUCLEO_SPIx_MISO_MOSI_AF;
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    HAL_GPIO_Init(NUCLEO_SPIx_MISO_MOSI_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = NUCLEO_SPIx_MISO_PIN;
    HAL_GPIO_Init(NUCLEO_SPIx_MISO_MOSI_GPIO_PORT, &GPIO_InitStruct);

    /*** Configure the SPI peripheral ***/
    /* Enable SPI clock */
    NUCLEO_SPIx_CLK_ENABLE();
  }
  //}}}
  //{{{
  static void SPIx_Init()
  {
    if(HAL_SPI_GetState(&hnucleo_Spi) == HAL_SPI_STATE_RESET)
    {
      /* SPI Config */
      hnucleo_Spi.Instance = NUCLEO_SPIx;
        /* SPI baudrate is set to 8 MHz maximum (PCLKx/SPI_BaudRatePrescaler = 32/4 = 8 MHz)
         to verify these constraints:
            - ST7735 LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
              Since the provided driver doesn't use read capability from LCD, only constraint
              on write baudrate is considered.
            - SD card SPI interface max baudrate is 25MHz for write/read
            - PCLK1 max frequency is 32 MHz
            - PCLK2 max frequency is 64 MHz
         */
      hnucleo_Spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
      hnucleo_Spi.Init.Direction = SPI_DIRECTION_2LINES;
      hnucleo_Spi.Init.CLKPhase = SPI_PHASE_2EDGE;
      hnucleo_Spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
      hnucleo_Spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
      hnucleo_Spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
      hnucleo_Spi.Init.CRCPolynomial = 7;
      hnucleo_Spi.Init.DataSize = SPI_DATASIZE_8BIT;
      hnucleo_Spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
      hnucleo_Spi.Init.NSS = SPI_NSS_SOFT;
      hnucleo_Spi.Init.TIMode = SPI_TIMODE_DISABLE;
      hnucleo_Spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
      hnucleo_Spi.Init.Mode = SPI_MODE_MASTER;

      SPIx_MspInit(&hnucleo_Spi);
      HAL_SPI_Init(&hnucleo_Spi);
    }
  }
  //}}}
  //{{{
  static void SPIx_WriteReadData (const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth)
  {
    HAL_StatusTypeDef status = HAL_OK;

    status = HAL_SPI_TransmitReceive(&hnucleo_Spi, (uint8_t*) DataIn, DataOut, DataLegnth, SpixTimeout);

    /* Check the communication status */
    if(status != HAL_OK)
    {
      /* Execute user timeout callback */
      SPIx_Error();
    }
  }
  //}}}
  //{{{
  static uint32_t SPIx_Read()
  {
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t readvalue = 0;
    uint32_t writevalue = 0xFFFFFFFF;

    status = HAL_SPI_TransmitReceive(&hnucleo_Spi, (uint8_t*) &writevalue, (uint8_t*) &readvalue, 1, SpixTimeout);

    /* Check the communication status */
    if(status != HAL_OK)
    {
      /* Execute user timeout callback */
      SPIx_Error();
    }

    return readvalue;
  }
  //}}}
  //{{{
  static void SPIx_Write (uint8_t Value)
  {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t data;

    status = HAL_SPI_TransmitReceive(&hnucleo_Spi, (uint8_t*) &Value, &data, 1, SpixTimeout);

    /* Check the communication status */
    if(status != HAL_OK)
    {
      /* Execute user timeout callback */
      SPIx_Error();
    }
  }
  //}}}
  //{{{
  static void SPIx_Error()
  {
    /* De-initialize the SPI communication BUS */
    HAL_SPI_DeInit(&hnucleo_Spi);

    /* Re-Initiaize the SPI communication BUS */
    SPIx_Init();
  }
  //}}}

  //{{{
  void SD_IO_Init()
  {
    GPIO_InitTypeDef  GPIO_InitStruct;
    uint8_t counter;

    /* SD_CS_GPIO Periph clock enable */
    SD_CS_GPIO_CLK_ENABLE();

    /* Configure SD_CS_PIN pin: SD Card CS pin */
    GPIO_InitStruct.Pin = SD_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SD_CS_GPIO_PORT, &GPIO_InitStruct);

    /*------------Put SD in SPI mode--------------*/
    /* SD SPI Config */
    SPIx_Init();

    /* SD chip select high */
    SD_CS_HIGH();

    /* Send dummy byte 0xFF, 10 times with CS high */
    /* Rise CS and MOSI for 80 clocks cycles */
    for (counter = 0; counter <= 9; counter++)
    {
      /* Send dummy byte 0xFF */
      SD_IO_WriteByte(SD_DUMMY_BYTE);
    }
  }
  //}}}
  //{{{
  void SD_IO_CSState (uint8_t val)
  {
    if(val == 1)
    {
      SD_CS_HIGH();
    }
    else
    {
      SD_CS_LOW();
    }
  }
  //}}}
  //{{{
  void SD_IO_WriteReadData (const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
  {
  //  /* SD chip select low */
  //  SD_CS_LOW();
    /* Send the byte */
    SPIx_WriteReadData(DataIn, DataOut, DataLength);
  }
  //}}}
  //{{{
  uint8_t SD_IO_ReadByte()
  {
    uint8_t data = 0;

    /* Get the received data */
    data = SPIx_Read();

    /* Return the shifted data */
    return data;
  }
  //}}}
  //{{{
  HAL_StatusTypeDef SD_IO_WriteCmd (uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t Response)
  {
    uint32_t counter = 0x00;
    uint8_t frame[6];

    /* Prepare Frame to send */
    frame[0] = (Cmd | 0x40);         /* Construct byte 1 */
    frame[1] = (uint8_t)(Arg >> 24); /* Construct byte 2 */
    frame[2] = (uint8_t)(Arg >> 16); /* Construct byte 3 */
    frame[3] = (uint8_t)(Arg >> 8);  /* Construct byte 4 */
    frame[4] = (uint8_t)(Arg);       /* Construct byte 5 */
    frame[5] = (Crc);                /* Construct byte 6 */

    /* SD chip select low */
    SD_CS_LOW();

    /* Send Frame */
    for (counter = 0; counter < 6; counter++)
    {
      SD_IO_WriteByte(frame[counter]); /* Send the Cmd bytes */
    }

    if(Response != SD_NO_RESPONSE_EXPECTED)
    {
      return SD_IO_WaitResponse(Response);
    }

    return HAL_OK;
  }
  //}}}
  //{{{
  HAL_StatusTypeDef SD_IO_WaitResponse (uint8_t Response)
  {
    uint32_t timeout = 0xFFFF;

    /* Check if response is got or a timeout is happen */
    while ((SD_IO_ReadByte() != Response) && timeout)
    {
      timeout--;
    }

    if (timeout == 0)
    {
      /* After time out */
      return HAL_TIMEOUT;
    }
    else
    {
      /* Right response got */
      return HAL_OK;
    }
  }
  //}}}
  //{{{
  uint8_t SD_IO_WriteByte (uint8_t Data)
  {
    uint8_t tmp;
    /* Send the byte */
    SPIx_WriteReadData(&Data,&tmp,1);
    return tmp;
  }

  /**
    * @brief  Sends dummy byte with CS High
    * @retval None
    */
  void SD_IO_WriteDummy(void)
  {
    /* SD chip select high */
    SD_CS_HIGH();

    /* Send Dummy byte 0xFF */
    SD_IO_WriteByte(SD_DUMMY_BYTE);
  }
  //}}}
  //}}}
#endif
