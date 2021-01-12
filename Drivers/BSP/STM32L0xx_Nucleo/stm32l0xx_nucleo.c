/**
  ******************************************************************************
  * @file    stm32l0xx_nucleo.c
  * @author  MCD Application Team
  * @brief   This file provides set of firmware functions to manage:
  *          - LEDs and push-button available on STM32L0XX-Nucleo Kit 
  *            from STMicroelectronics
  *          - LCD, joystick and microSD available on Adafruit 1.8" TFT LCD 
  *            shield (reference ID 802)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright(c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */ 
  
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_nucleo.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32L0XX_NUCLEO
  * @{
  */   
    
/** @addtogroup STM32L0XX_NUCLEO_LOW_LEVEL 
  * @brief This file provides set of firmware functions to manage Leds and push-button
  *        available on STM32L0XX-Nucleo Kit from STMicroelectronics.
  * @{
  */ 

/** @defgroup STM32L0XX_NUCLEO_LOW_LEVEL_Private_TypesDefinitions Private Types Definitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM32L0XX_NUCLEO_LOW_LEVEL_Private_Defines Private Defines
  * @{
  */ 

/**
  * @brief STM32L0XX NUCLEO BSP Driver version number
  */
#define __STM32L0XX_NUCLEO_BSP_VERSION_MAIN   (0x02) /*!< [31:24] main version */
#define __STM32L0XX_NUCLEO_BSP_VERSION_SUB1   (0x01) /*!< [23:16] sub1 version */
#define __STM32L0XX_NUCLEO_BSP_VERSION_SUB2   (0x03) /*!< [15:8]  sub2 version */
#define __STM32L0XX_NUCLEO_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32L0XX_NUCLEO_BSP_VERSION         ((__STM32L0XX_NUCLEO_BSP_VERSION_MAIN << 24)\
                                             |(__STM32L0XX_NUCLEO_BSP_VERSION_SUB1 << 16)\
                                             |(__STM32L0XX_NUCLEO_BSP_VERSION_SUB2 << 8 )\
                                             |(__STM32L0XX_NUCLEO_BSP_VERSION_RC))

/**
  * @brief LINK SD Card
  */
#define SD_DUMMY_BYTE            0xFF    
#define SD_NO_RESPONSE_EXPECTED  0x80

/**
  * @}
  */ 


/** @defgroup STM32L0XX_NUCLEO_LOW_LEVEL_Private_Variables Private Variables
  * @{
  */ 
GPIO_TypeDef* LED_PORT[LEDn] = {LED2_GPIO_PORT};

const uint16_t LED_PIN[LEDn] = {LED2_PIN};
                                
GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT }; 
const uint16_t BUTTON_PIN[BUTTONn] = {USER_BUTTON_PIN }; 
const uint8_t BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn };

/**
 * @brief BUS variables
 */

#ifdef HAL_SPI_MODULE_ENABLED
uint32_t SpixTimeout = NUCLEO_SPIx_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */
static SPI_HandleTypeDef hnucleo_Spi;
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
uint32_t I2cxTimeout = NUCLEO_I2C1_TIMEOUT_MAX; /*<! Value of Timeout when I2C communication fails */
static I2C_HandleTypeDef hnucleo_I2c1;
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
static ADC_HandleTypeDef hnucleo_Adc;
/* ADC channel configuration structure declaration */
static ADC_ChannelConfTypeDef sConfig;
#endif /* HAL_ADC_MODULE_ENABLED */

/**
  * @}
  */ 

/** @defgroup STM32L0XX_NUCLEO_LOW_LEVEL_Private_FunctionPrototypes Private Function Prototypes
  * @{
  */ 
#ifdef HAL_SPI_MODULE_ENABLED
static void               SPIx_Init(void);
static void               SPIx_Write(uint8_t Value);
static void               SPIx_WriteData(uint8_t *DataIn, uint16_t DataLength);
static void               SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth);
static void               SPIx_Error (void);
static void               SPIx_MspInit(void);

/* SD IO functions */
void                      SD_IO_Init(void);
void                      SD_IO_CSState(uint8_t state);
void                      SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength);
void                      SD_IO_ReadData(uint8_t *DataOut, uint16_t DataLength);
void                      SD_IO_WriteData(const uint8_t *Data, uint16_t DataLength);
uint8_t                   SD_IO_WriteByte(uint8_t Data);
uint8_t                   SD_IO_ReadByte(void);

/* LCD IO functions */
void                      LCD_IO_Init(void);
void                      LCD_IO_WriteData(uint8_t Data);
void                      LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size);
void                      LCD_IO_WriteReg(uint8_t LCDReg);
void                      LCD_Delay(uint32_t delay);
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
/* I2C1 bus function */
/* Link function for I2C SEESAW peripheral */
static void               I2C1_Init(void);
static void               I2C1_DeInit(void);
static void               I2C1_Error(void);
static void               I2C1_MspInit(I2C_HandleTypeDef* hi2c);
static void               I2C1_MspDeInit(I2C_HandleTypeDef* hi2c);
static void               I2C1_Write(uint8_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t Value);
static uint8_t            I2C1_Read(uint8_t Addr, uint16_t Reg, uint16_t RegSize);
static HAL_StatusTypeDef  I2C1_WriteBuffer(uint8_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t* pBuffer, uint16_t Length);
static HAL_StatusTypeDef  I2C1_ReadBuffer(uint8_t Addr, uint8_t* pBuffer, uint16_t Length);
static HAL_StatusTypeDef  I2C1_SendAddr(uint8_t Addr, uint16_t Reg, uint16_t RegSize);
static HAL_StatusTypeDef  I2C1_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);

#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
static HAL_StatusTypeDef               ADCx_Init(void);
static void               ADCx_DeInit(void);
static void               ADCx_MspInit(ADC_HandleTypeDef *hadc);
static void               ADCx_MspDeInit(ADC_HandleTypeDef *hadc);
#endif /* HAL_ADC_MODULE_ENABLED */
/**
  * @}
  */ 

/** @defgroup STM32L0XX_NUCLEO_LOW_LEVEL_Private_Functions Private Functions
  * @{
  */ 

/**
  * @brief  This method returns the STM32L0XX NUCLEO BSP Driver revision
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __STM32L0XX_NUCLEO_BSP_VERSION;
}

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Led to be configured. 
  *          This parameter can be one of the following values:
  *            @arg  LED2
  * @retval None
  */
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpioinitstruct;
  
  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  gpioinitstruct.Pin = LED_PIN[Led];
  gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull = GPIO_NOPULL;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  
  HAL_GPIO_Init(LED_PORT[Led], &gpioinitstruct);

  /* Reset PIN to switch off the LED */
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  DeInit LEDs.
  * @param  Led: LED to be de-init. 
  *   This parameter can be one of the following values:
  *     @arg  LED2
  * @note Led DeInit does not disable the GPIO clock nor disable the Mfx 
  * @retval None
  */
void BSP_LED_DeInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* Turn off LED */
  HAL_GPIO_WritePin(LED_PORT[Led],LED_PIN[Led], GPIO_PIN_RESET);
  /* DeInit the GPIO_LED pin */
  gpio_init_structure.Pin = LED_PIN[Led];
  HAL_GPIO_DeInit(LED_PORT[Led], gpio_init_structure.Pin);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET); 
}

/**
  * @brief  Turns selected LED Off. 
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_USER
  * @param  ButtonMode: Specifies Button mode.
  *   This parameter can be one of following parameters:   
  *     @arg  BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability  
  * @retval None
  */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef gpioinitstruct;
  
  /* Enable the BUTTON Clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);

  gpioinitstruct.Pin = BUTTON_PIN[Button];
  gpioinitstruct.Pull = GPIO_NOPULL;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  
  if(ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    gpioinitstruct.Mode = GPIO_MODE_INPUT;
  
    HAL_GPIO_Init(BUTTON_PORT[Button], &gpioinitstruct);
  }
  
  if(ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    gpioinitstruct.Mode   = GPIO_MODE_IT_FALLING; 
    HAL_GPIO_Init(BUTTON_PORT[Button], &gpioinitstruct);
    
    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}

/**
  * @brief  Push Button DeInit.
  * @param  Button: Button to be configured
  *   This parameter should be: BUTTON_USER  
  * @note PB DeInit does not disable the GPIO clock
  * @retval None
  */
void BSP_PB_DeInit(Button_TypeDef Button)
{
  GPIO_InitTypeDef gpio_init_structure;

  gpio_init_structure.Pin = BUTTON_PIN[Button];
  HAL_NVIC_DisableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  HAL_GPIO_DeInit(BUTTON_PORT[Button], gpio_init_structure.Pin);
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_USER
  * @retval Button state.
  */
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/
#if defined(HAL_I2C_MODULE_ENABLED)
/******************************* I2C Routines *********************************/

/**
  * @brief I2C Bus initialization
  * @param None
  * @retval None
  */
static void I2C1_Init(void)
{
    if (HAL_I2C_GetState(&hnucleo_I2c1) == HAL_I2C_STATE_RESET)
    {
        hnucleo_I2c1.Instance = NUCLEO_I2C1;
        hnucleo_I2c1.Init.Timing = I2C1_TIMING;
        hnucleo_I2c1.Init.OwnAddress1 = 0;
        hnucleo_I2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
        hnucleo_I2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
        hnucleo_I2c1.Init.OwnAddress2 = 0;
        hnucleo_I2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
        hnucleo_I2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
        hnucleo_I2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

        /* Init the I2C */
        I2C1_MspInit(&hnucleo_I2c1);
        HAL_I2C_Init(&hnucleo_I2c1);
    }
}

/**
  * @brief I2C1 Bus Deinitialization
  * @retval None
  */
static void I2C1_DeInit(void)
{
    if (HAL_I2C_GetState(&hnucleo_I2c1) != HAL_I2C_STATE_RESET)
    {
        /* DeInit the I2C */
        HAL_I2C_DeInit(&hnucleo_I2c1);
        I2C1_MspDeInit(&hnucleo_I2c1);
    }
}
/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Value: Data to be written
  * @retval None
  */
static void I2C1_Write(uint8_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t Value)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Shift address left per HAL API requirement */
    Addr <<= 1;

    status = HAL_I2C_Mem_Write(&hnucleo_I2c1, Addr, Reg, RegSize, &Value, 1, 100);

    /* Check the communication status */
    if (status != HAL_OK)
    {
        /* Execute user timeout callback */
        I2C1_Error();
    }
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @retval Read data
  */
static uint8_t I2C1_Read(uint8_t Addr, uint16_t Reg, uint16_t RegSize)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t Value = 0;

    /* Shift address left per HAL API requirement */
    Addr <<= 1;

    status = HAL_I2C_Mem_Read(&hnucleo_I2c1, Addr, Reg, RegSize, &Value, 1, 1000);

    /* Check the communication status */
    if (status != HAL_OK)
    {
        /* Execute user timeout callback */
        I2C1_Error();
    }
    return Value;
}


/**
  * @brief  Sends address I2C transaction with STOP.
  * @note   Used in conjunction with I2C1_ReadBuffer to follow up with the read.
  * @param  Addr  : I2C Address
  * @param  Reg   : Reg Address
  * @param  RegSize : The target register size (can be 8BIT or 16BIT)
  * @retval 0 if no problems to send address
  */
static HAL_StatusTypeDef I2C1_SendAddr(uint8_t Addr, uint16_t Reg, uint16_t RegSize)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Shift address left per HAL API requirement */
    Addr <<= 1;

    /* Master transaction to set register address. */
    uint8_t buf[2];
    uint16_t size;
    
    if (RegSize == I2C_MEMADD_SIZE_16BIT)
    {
        buf[0] = Reg >> 8;
        buf[1] = Reg;
        size = 2;
    }
    else
    {
        buf[0] = Reg;
        size = 1;
    }    
    
    status = HAL_I2C_Master_Transmit(&hnucleo_I2c1, Addr, buf, size, I2cxTimeout);

    /* Check the communication status */
    if (status != HAL_OK)
    {
        /* Re-Initiaize the BUS */
        I2C1_Error();
    }
    return status;
}

/**
  * @brief  Reads multiple data on the BUS.
  * @note   To be preceded by I2C1_SendAddr
  * @param  Addr  : I2C Address
  * @param  pBuffer : pointer to read data buffer
  * @param  Length : length of the data
  * @retval 0 if no problems to read multiple data
  */
static HAL_StatusTypeDef I2C1_ReadBuffer(uint8_t Addr, uint8_t* pBuffer, uint16_t Length)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Shift address left per HAL API requirement */
    Addr <<= 1;

    /* Master transaction to receive data, having already sent register address. */
    status = HAL_I2C_Master_Receive(&hnucleo_I2c1, Addr, pBuffer, Length,
       I2cxTimeout);

    /* Check the communication status */
    if (status != HAL_OK)
    {
        /* Re-Initiaize the BUS */
        I2C1_Error();
    }
    return status;
}

/**
  * @brief  Checks if target device is ready for communication.
  * @note   This function is used with Memory devices
  * @param  DevAddress: Target device address
  * @param  Trials: Number of trials
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C1_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{
    /* Shift address left per HAL API requirement */
    DevAddress <<= 1;

    return (HAL_I2C_IsDeviceReady(&hnucleo_I2c1, DevAddress, Trials, I2cxTimeout));
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.
  * @param  Reg: The target register address to write
  * @param  RegSize: The target register size (can be 8BIT or 16BIT)
  * @param  pBuffer: The target register value to be written
  * @param  Length: buffer size to be written
  * @retval None
  */
static HAL_StatusTypeDef I2C1_WriteBuffer(uint8_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t* pBuffer, uint16_t Length)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Shift address left per HAL API requirement */
    Addr <<= 1;

    status = HAL_I2C_Mem_Write(&hnucleo_I2c1, Addr, Reg, RegSize, pBuffer, Length, I2cxTimeout);

    /* Check the communication status */
    if (status != HAL_OK)
    {
        /* Re-Initiaize the BUS */
        I2C1_Error();
    }
    return status;
}

/**
  * @brief  Manages error callback by re-initializing I2C.
  * @param  None
  * @retval None
  */
static void I2C1_Error(void)
{
    /* De-initialize the I2C communication BUS */
    HAL_I2C_DeInit(&hnucleo_I2c1);

    /* Re-Initiaize the I2C communication BUS */
    I2C1_Init();
}

/**
  * @brief I2C MSP Initialization
  * @param hi2c: I2C handle
  * @retval None
  */
static void I2C1_MspInit(I2C_HandleTypeDef* hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;

    /*##-1- Set source clock to SYSCLK for I2C1 ################################################*/
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /*##-2- Configure the GPIOs ################################################*/

    /* Enable GPIO clock */
    NUCLEO_I2C1_GPIO_CLK_ENABLE();

    /* Configure I2C SCL & SDA as alternate function  */
    GPIO_InitStruct.Pin = (NUCLEO_I2C1_SCL_PIN | NUCLEO_I2C1_SDA_PIN);
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = NUCLEO_I2C1_SCL_SDA_AF;
    HAL_GPIO_Init(NUCLEO_I2C1_GPIO_PORT, &GPIO_InitStruct);

    /*##-3- Configure the Eval I2C peripheral #######################################*/
    /* Enable I2C clock */
    NUCLEO_I2C1_CLK_ENABLE();

    /* Force the I2C peripheral clock reset */
    NUCLEO_I2C1_FORCE_RESET();

    /* Release the I2C peripheral clock reset */
    NUCLEO_I2C1_RELEASE_RESET();
}

/**
  * @brief I2C1 MSP DeInitialization
  * @param hi2c: I2C1 handle
  * @retval None
  */
static void I2C1_MspDeInit(I2C_HandleTypeDef* hi2c)
{

    /*##-1- Unconfigure the GPIOs ################################################*/
    /* Enable GPIO clock */
    NUCLEO_I2C1_GPIO_CLK_ENABLE();

    /* Configure I2C Rx/Tx as alternate function  */
    HAL_GPIO_DeInit(NUCLEO_I2C1_GPIO_PORT, (NUCLEO_I2C1_SCL_PIN | NUCLEO_I2C1_SDA_PIN));

    /*##-2- Unconfigure the Nucleo I2C1 peripheral ############################*/
    /* Force and release I2C Peripheral */
    NUCLEO_I2C1_FORCE_RESET();
    NUCLEO_I2C1_RELEASE_RESET();

    /* Disable Nucleo I2C1 clock */
    NUCLEO_I2C1_RELEASE_RESET();
}

#endif /*HAL_I2C_MODULE_ENABLED*/

#ifdef HAL_SPI_MODULE_ENABLED
/******************************* SPI Routines *********************************/
/**
  * @brief  Initialize SPI MSP.
  * @retval None
  */
static void SPIx_MspInit(void)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};
  
  /*** Configure the GPIOs ***/  
  /* Enable GPIO clock */
  NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE();
  NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE();
  
  /* Configure SPI SCK */
  gpioinitstruct.Pin = NUCLEO_SPIx_SCK_PIN;
  gpioinitstruct.Mode = GPIO_MODE_AF_PP;
  gpioinitstruct.Pull  = GPIO_PULLUP;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpioinitstruct.Alternate = NUCLEO_SPIx_SCK_AF;
  HAL_GPIO_Init(NUCLEO_SPIx_SCK_GPIO_PORT, &gpioinitstruct);

  /* Configure SPI MISO and MOSI */ 
  gpioinitstruct.Pin = NUCLEO_SPIx_MOSI_PIN;
  gpioinitstruct.Alternate = NUCLEO_SPIx_MISO_MOSI_AF;
  gpioinitstruct.Pull  = GPIO_PULLDOWN;
  HAL_GPIO_Init(NUCLEO_SPIx_MISO_MOSI_GPIO_PORT, &gpioinitstruct);
  
  gpioinitstruct.Pin = NUCLEO_SPIx_MISO_PIN;
  HAL_GPIO_Init(NUCLEO_SPIx_MISO_MOSI_GPIO_PORT, &gpioinitstruct);

  /*** Configure the SPI peripheral ***/ 
  /* Enable SPI clock */
  NUCLEO_SPIx_CLK_ENABLE();
}

/**
  * @brief  Initialize SPI HAL.
  * @retval None
  */
static void SPIx_Init(void)
{
  if(HAL_SPI_GetState(&hnucleo_Spi) == HAL_SPI_STATE_RESET)
  {
    /* SPI Config */
    hnucleo_Spi.Instance = NUCLEO_SPIx;
      /* SPI baudrate is set to 8 MHz maximum (PCLK2/SPI_BaudRatePrescaler = 32/4 = 8 MHz) 
       to verify these constraints:
          - ST7735 LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
            Since the provided driver doesn't use read capability from LCD, only constraint 
            on write baudrate is considered.
          - SD card SPI interface max baudrate is 25MHz for write/read
          - PCLK2 max frequency is 32 MHz 
       */ 
    hnucleo_Spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hnucleo_Spi.Init.Direction = SPI_DIRECTION_2LINES;
    hnucleo_Spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    hnucleo_Spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    hnucleo_Spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hnucleo_Spi.Init.CRCPolynomial = 7;
    hnucleo_Spi.Init.DataSize = SPI_DATASIZE_8BIT;
    hnucleo_Spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hnucleo_Spi.Init.NSS = SPI_NSS_SOFT;
    hnucleo_Spi.Init.TIMode = SPI_TIMODE_DISABLE;
    hnucleo_Spi.Init.Mode = SPI_MODE_MASTER;
    
    SPIx_MspInit();
    HAL_SPI_Init(&hnucleo_Spi);
  }
}

/**
  * @brief  SPI Write a byte to device
  * @param  DataIn: value to be written
  * @param  DataOut: read value
  * @param  DataLength: number of bytes to write
  * @retval None
  */
static void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_TransmitReceive(&hnucleo_Spi, (uint8_t*) DataIn, DataOut, DataLength, SpixTimeout);
    
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}

/**
  * @brief  SPI Write an amount of data to device
  * @param  DataIn: value to be written
  * @param  DataLength: number of bytes to write
  * @retval None
  */
static void SPIx_WriteData(uint8_t *DataIn, uint16_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Transmit(&hnucleo_Spi, DataIn, DataLength, SpixTimeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}

/**
  * @brief  SPI Write a byte to device
  * @param  Value: value to be written
  * @retval None
  */
static void SPIx_Write(uint8_t Value)
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

/**
  * @brief  SPI error treatment function
  * @retval None
  */
static void SPIx_Error (void)
{
  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&hnucleo_Spi);
  
  /* Re-Initiaize the SPI communication BUS */
  SPIx_Init();
}

/******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/********************************* LINK SD ************************************/
/**
  * @brief  Initialize the SD Card and put it into StandBy State (Ready for 
  *         data transfer).
  * @retval None
  */
void SD_IO_Init(void)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};
  uint8_t counter = 0;

  /* SD_CS_GPIO Periph clock enable */
  SD_CS_GPIO_CLK_ENABLE();

  /* Configure SD_CS_PIN pin: SD Card CS pin */
  gpioinitstruct.Pin = SD_CS_PIN;
  gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull = GPIO_PULLUP;
  gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_PORT, &gpioinitstruct);

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

/**
  * @brief  Set the SD_CS pin.
  * @param  val: pin value.
  * @retval None
  */
void SD_IO_CSState(uint8_t val)
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

/**
  * @brief  Write byte(s) on the SD
  * @param  DataIn: Pointer to data buffer to write
  * @param  DataOut: Pointer to data buffer for read data
  * @param  DataLength: number of bytes to write
  * @retval None
  */
void SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
  /* Send the byte */
  SPIx_WriteReadData(DataIn, DataOut, DataLength);
}

/**
  * @brief  Write a byte on the SD.
  * @param  Data: byte to send.
  * @retval Data written
  */
uint8_t SD_IO_WriteByte(uint8_t Data)
{
  uint8_t tmp;

  /* Send the byte */
  SPIx_WriteReadData(&Data,&tmp,1);
  return tmp;
}

/**
  * @brief  Write an amount of data on the SD.
  * @param  DataOut: byte to send.
  * @param  DataLength: number of bytes to write
  * @retval none
  */
void SD_IO_ReadData(uint8_t *DataOut, uint16_t DataLength)
{
  /* Send the byte */
  SD_IO_WriteReadData(DataOut, DataOut, DataLength);
  }  
 
/**
  * @brief  Write an amount of data on the SD.
  * @param  Data: byte to send.
  * @param  DataLength: number of bytes to write
  * @retval none
  */
void SD_IO_WriteData(const uint8_t *Data, uint16_t DataLength)
{
  /* Send the byte */
  SPIx_WriteData((uint8_t *)Data, DataLength);
}

/********************************* LINK LCD ***********************************/
/**
  * @brief  Initialize the LCD
  * @retval None
  */
void LCD_IO_Init(void)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};
   
  /* LCD_CS_GPIO and LCD_DC_GPIO Periph clock enable */
  LCD_CS_GPIO_CLK_ENABLE();
  LCD_DC_GPIO_CLK_ENABLE();
  
  /* Configure LCD_CS_PIN pin: LCD Card CS pin */
  gpioinitstruct.Pin = LCD_CS_PIN;
  gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull = GPIO_NOPULL;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_PORT, &gpioinitstruct);

  /* Configure LCD_DC_PIN pin: LCD Card DC pin */
  gpioinitstruct.Pin = LCD_DC_PIN;
  HAL_GPIO_Init(LCD_DC_GPIO_PORT, &gpioinitstruct);
  
  /* LCD chip select high */
  LCD_CS_HIGH();
  
  /* LCD SPI Config */
  SPIx_Init();
}

/**
  * @brief  Write command to select the LCD register.
  * @param  LCDReg: Address of the selected register.
  * @retval None
  */
void LCD_IO_WriteReg(uint8_t LCDReg)
{
  /* Reset LCD control line CS */
  LCD_CS_LOW();
  
  /* Set LCD data/command line DC to Low */
  LCD_DC_LOW();
    
  /* Send Command */
  SPIx_Write(LCDReg);
  
  /* Deselect : Chip Select high */
  LCD_CS_HIGH();
}

/**
  * @brief  Writes data to select the LCD register.
  *         This function must be used after st7735_WriteReg() function
  * @param  Data: data to write to the selected register.
  * @retval None
  */
void LCD_IO_WriteData(uint8_t Data)
{
  /* Reset LCD control line CS */
  LCD_CS_LOW();
  
  /* Set LCD data/command line DC to High */
  LCD_DC_HIGH();

  /* Send Data */
  SPIx_Write(Data);
  
  /* Deselect : Chip Select high */
  LCD_CS_HIGH();
}

/**
* @brief  Write register value.
* @param  pData Pointer on the register value
* @param  Size Size of byte to transmit to the register
* @retval None
*/
void LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size)
{
  uint32_t counter = 0;
  __IO uint32_t data = 0;
  
  /* Reset LCD control line CS */
  LCD_CS_LOW();
  
  /* Set LCD data/command line DC to High */
  LCD_DC_HIGH();

  if (Size == 1)
  {
    /* Only 1 byte to be sent to LCD - general interface can be used */
    /* Send Data */
    SPIx_Write(*pData);
  }
  else
  {
    /* Several data should be sent in a raw */
    /* Direct SPI accesses for optimization */
    for (counter = Size; counter != 0; counter--)
    {
      while(((hnucleo_Spi.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
      {
      }  
      /* Need to invert bytes for LCD*/
      *((__IO uint8_t*)&hnucleo_Spi.Instance->DR) = *(pData+1);
      
      while(((hnucleo_Spi.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
      {
      }  
      *((__IO uint8_t*)&hnucleo_Spi.Instance->DR) = *pData;
      counter--;
      pData += 2;
    }
    
    /* Wait until the bus is ready before releasing Chip select */ 
    while(((hnucleo_Spi.Instance->SR) & SPI_FLAG_BSY) != RESET)
    {
    }  
  } 

  /* Empty the Rx fifo */
  data = *(&hnucleo_Spi.Instance->DR);
  UNUSED(data);  /* Remove GNU warning */

  /* Deselect : Chip Select high */
  LCD_CS_HIGH();
}

/**
  * @brief  Wait for loop in ms.
  * @param  Delay in ms.
  * @retval None
  */
void LCD_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}
#endif /* HAL_SPI_MODULE_ENABLED */

/********************************* LINK SEESAW ********************************/

#if defined(HAL_I2C_MODULE_ENABLED) && defined(USE_ADAFRUIT_SHIELD_V2)

/**
 * @brief  SEESAW delay
 * @param  Delay: Delay in ms
 * @retval None
 */
void SEESAW_IO_Delay(uint32_t Delay)
{
    HAL_Delay(Delay);
}

void SEESAW_IO_DelayMicroseconds(uint32_t Delay_us)
{
    /* Round up to nearest millisecond. */

    uint32_t delay_ms = ((Delay_us)+999) / 1000;
    HAL_Delay(delay_ms);
}

/**
  * @brief  Initializes IOE low level.
  * @param  None
  * @retval None
  */
void SEESAW_IO_Init(void)
{
    I2C1_Init();
}
/**
  * @brief  SEESAW writes single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Value: Data to be written
  * @retval None
  */
void SEESAW_IO_Write(uint8_t Addr, uint16_t Reg, uint8_t Value)
{
    I2C1_Write(Addr, Reg, I2C_MEMADD_SIZE_16BIT, Value);
}

/**
  * @brief  SEESAW reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @retval Read data
  */
uint8_t SEESAW_IO_Read(uint8_t Addr, uint16_t Reg)
{
    return I2C1_Read(Addr, Reg, I2C_MEMADD_SIZE_16BIT);
}

/**
  * @brief  SEESAW reads multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
uint16_t SEESAW_IO_ReadMultiple(uint8_t Addr, uint16_t Reg, uint8_t* Buffer, uint16_t Length, uint16_t Delay_us)
{
    HAL_StatusTypeDef status;

    status = I2C1_SendAddr(Addr, Reg, I2C_MEMADD_SIZE_16BIT);

    if (status == HAL_OK)
    {
        SEESAW_IO_DelayMicroseconds(Delay_us);

        status = I2C1_ReadBuffer(Addr, Buffer, Length);
    }

    return status;
}

/**
  * @brief  SEESAW  writes multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval None
  */

void SEESAW_IO_WriteMultiple(uint8_t Addr, uint16_t Reg, uint8_t *Buffer, uint16_t Length)
{
    I2C1_WriteBuffer(Addr, Reg, I2C_MEMADD_SIZE_16BIT, Buffer, Length);
}




/**
  * @brief  Checks if Temperature Sensor is ready for communication.
  * @param  DevAddress: Target device address
  * @param  Trials: Number of trials
  * @retval HAL status
  */
uint16_t SEESAW_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{
    return (I2C1_IsDeviceReady(DevAddress, Trials));
}

#endif /* HAL_I2C_MODULE_ENABLED */

/******************************* LINK JOYSTICK ********************************/
#ifdef HAL_ADC_MODULE_ENABLED
/**
  * @brief  Initialize ADC MSP.
  * @retval None
  */
static void ADCx_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};
  
  /*** Configure the GPIOs ***/  
  /* Enable GPIO clock */
  NUCLEO_ADCx_GPIO_CLK_ENABLE();
  
  /* Configure ADC1 Channel8 as analog input */
  gpioinitstruct.Pin = NUCLEO_ADCx_GPIO_PIN ;
  gpioinitstruct.Mode = GPIO_MODE_ANALOG;
  gpioinitstruct.Pull   = GPIO_NOPULL;
  HAL_GPIO_Init(NUCLEO_ADCx_GPIO_PORT, &gpioinitstruct);

  /*** Configure the ADC peripheral ***/ 
  /* Enable ADC clock */
  NUCLEO_ADCx_CLK_ENABLE(); 
}

/**
  * @brief  DeInitializes ADC MSP.
  * @param  hadc: ADC peripheral
  * @note ADC DeInit does not disable the GPIO clock
  * @retval None
  */
static void ADCx_MspDeInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef  gpioinitstruct;

  /*** DeInit the ADC peripheral ***/ 
  /* Disable ADC clock */
  NUCLEO_ADCx_CLK_DISABLE(); 

  /* Configure the selected ADC Channel as analog input */
  gpioinitstruct.Pin = NUCLEO_ADCx_GPIO_PIN ;
  HAL_GPIO_DeInit(NUCLEO_ADCx_GPIO_PORT, gpioinitstruct.Pin);

  /* Disable GPIO clock has to be done by the application*/
  /* NUCLEO_ADCx_GPIO_CLK_DISABLE(); */
}

/**
  * @brief  Initializes ADC HAL.
  * @retval None
  */
static HAL_StatusTypeDef ADCx_Init(void)
{
  /* Set ADC instance */
  hnucleo_Adc.Instance = NUCLEO_ADCx;

  if(HAL_ADC_GetState(&hnucleo_Adc) == HAL_ADC_STATE_RESET)
  {
    /* ADC Config */
    hnucleo_Adc.Instance = NUCLEO_ADCx;
    hnucleo_Adc.Init.OversamplingMode      = DISABLE;
    hnucleo_Adc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2; /* (must not exceed 16MHz) */
    hnucleo_Adc.Init.LowPowerAutoPowerOff  = DISABLE;
    hnucleo_Adc.Init.LowPowerFrequencyMode = ENABLE;
    hnucleo_Adc.Init.LowPowerAutoWait      = ENABLE;
    hnucleo_Adc.Init.Resolution            = ADC_RESOLUTION_12B;
    hnucleo_Adc.Init.SamplingTime          = ADC_SAMPLETIME_1CYCLE_5;
    hnucleo_Adc.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;
    hnucleo_Adc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hnucleo_Adc.Init.ContinuousConvMode    = DISABLE;
    hnucleo_Adc.Init.DiscontinuousConvMode = DISABLE;
    hnucleo_Adc.Init.ExternalTrigConv       = ADC_SOFTWARE_START;            /* Trig of conversion start done manually by software, without external event */
    hnucleo_Adc.Init.ExternalTrigConvEdge   = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because trig by software start */
    hnucleo_Adc.Init.EOCSelection           = ADC_EOC_SEQ_CONV;
    hnucleo_Adc.Init.DMAContinuousRequests  = DISABLE;
    
    /* Initialize MSP related to ADC */
    ADCx_MspInit(&hnucleo_Adc);
    
    /* Initialize ADC */
    if (HAL_ADC_Init(&hnucleo_Adc) != HAL_OK)
    {
      return HAL_ERROR;
    }
    
    if (HAL_ADCEx_Calibration_Start(&hnucleo_Adc,ADC_SINGLE_ENDED) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

/**
  * @brief  Initializes ADC HAL.
  * @retval None
  */
static void ADCx_DeInit(void)
{
    hnucleo_Adc.Instance   = NUCLEO_ADCx;
    
    HAL_ADC_DeInit(&hnucleo_Adc);
    ADCx_MspDeInit(&hnucleo_Adc);
}

#endif /* HAL_ADC_MODULE_ENABLED */

/******************************* LINK JOYSTICK ********************************/

#if USE_ADAFRUIT_SHIELD_V2
#include "stm32_adafruit_shield.h"

/**
  * @brief  Configures joystick available on adafruit 1.8" TFT shield V2
  *         managed through seesaw.
  * @retval Joystickstatus (0=> success, 1=> fail)
  */
uint8_t BSP_JOY_Init(void)
{
    /* Note: don't do a software reset of seesaw - this should 
     * have been done with LCD module init 
     */
    if (Adafruit_seesaw_init(TFTSHIELD_ADDR, false) != true)
    {
        return (uint8_t)HAL_ERROR;
    }
    return HAL_OK;
}

/**
  * @brief  DeInit joystick GPIOs.
  * @retval None.
  */
void BSP_JOY_DeInit(void)
{
    /* Do nothing */
}

/**
  * @brief  Returns the Joystick key pressed.
  * @note   To know which Joystick key is pressed we need to read seesaw.
  * @retval JOYState_TypeDef: Code of the Joystick key pressed.
  */
JOYState_TypeDef BSP_JOY_GetState(void)
{
    JOYState_TypeDef state = JOY_NONE;
    uint32_t  keyconvertedvalue = 0;

    /* Start the conversion process */
    keyconvertedvalue = Adafruit_seesaw_digitalReadBulk(TFTSHIELD_BUTTON_ALL);

#if 0 /* diagnostics */
    if (keyconvertedvalue != 0) {
        char buf[12];
        sprintf(buf, "%08x", keyconvertedvalue);
        BSP_LCD_DisplayStringAtLine(12, (uint8_t*)buf);
    }
#endif

    if ((keyconvertedvalue & TFTSHIELD_BUTTON_UP) == 0)
    {
        state = JOY_UP;
    }
    else if ((keyconvertedvalue & TFTSHIELD_BUTTON_RIGHT) == 0)
    {
        state = JOY_RIGHT;
    }
    else if ((keyconvertedvalue & TFTSHIELD_BUTTON_IN) == 0)
    {
        state = JOY_SEL;
    }
    else if ((keyconvertedvalue & TFTSHIELD_BUTTON_DOWN) == 0)
    {
        state = JOY_DOWN;
    }
    else if ((keyconvertedvalue & TFTSHIELD_BUTTON_LEFT) == 0)
    {
        state = JOY_LEFT;
    }
    else
    {
        state = JOY_NONE;
    }

    /* Return the code of the Joystick key pressed */
    return state;
}

#else /* USE_ADAFRUIT_SHIELD_V2 */

#ifdef HAL_ADC_MODULE_ENABLED
/**
  * @brief  Configures joystick available on adafruit 1.8" TFT shield 
  *         managed through ADC to detect motion.
  * @retval Joystickstatus (0=> success, 1=> fail) 
  */
uint8_t BSP_JOY_Init(void)
{
  if (ADCx_Init() != HAL_OK)
  {
    return (uint8_t) HAL_ERROR; 
  }
  
  /* Select Channel 8 to be converted */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;

  /* Return Joystick initialization status */
  return (uint8_t) HAL_ADC_ConfigChannel(&hnucleo_Adc, &sConfig);
}

/**
  * @brief  DeInit joystick GPIOs.
  * @note   JOY DeInit does not disable the Mfx, just set the Mfx pins in Off mode
  * @retval None.
  */
void BSP_JOY_DeInit(void)
{
    ADCx_DeInit();
}

/**
  * @brief  Returns the Joystick key pressed.
  * @note   To know which Joystick key is pressed we need to detect the voltage
  *         level on each key output
  *           - None  : 3.3 V / 4095
  *           - SEL   : 1.055 V / 1308
  *           - DOWN  : 0.71 V / 88
  *           - LEFT  : 3.0 V / 3720 
  *           - RIGHT : 0.595 V / 737
  *           - UP    : 1.65 V / 2046
  * @retval JOYState_TypeDef: Code of the Joystick key pressed.
  */
JOYState_TypeDef BSP_JOY_GetState(void)
{
  JOYState_TypeDef state = JOY_NONE;
  uint16_t  keyconvertedvalue = 0;
  
 /* Start the conversion process */
  HAL_ADC_Start(&hnucleo_Adc);
  
  /* Wait for the end of conversion */
  if (HAL_ADC_PollForConversion(&hnucleo_Adc, 10) != HAL_TIMEOUT)
  {
    /* Get the converted value of regular channel */
    keyconvertedvalue = HAL_ADC_GetValue(&hnucleo_Adc);
  }
 
  if((keyconvertedvalue > 2010) && (keyconvertedvalue < 2090))
  {
    state = JOY_UP;
  }
  else if((keyconvertedvalue > 680) && (keyconvertedvalue < 780))
  {
    state = JOY_RIGHT;
  }
  else if((keyconvertedvalue > 1270) && (keyconvertedvalue < 1350))
  {
    state = JOY_SEL;
  }
  else if((keyconvertedvalue > 50) && (keyconvertedvalue < 130))
  {
    state = JOY_DOWN;
  }
  else if((keyconvertedvalue > 3570) && (keyconvertedvalue < 3800))
  {
    state = JOY_LEFT;
  }
  else
  {
    state = JOY_NONE;
  }
  
  /* Return the code of the Joystick key pressed */
  return state;
}
#endif /* HAL_ADC_MODULE_ENABLED */

#endif /* USE_ADAFRUIT_SHIELD_V2 */

/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */    

/**
  * @}
  */ 
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
