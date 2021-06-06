/*****************************************************************************************************
* FILENAME :        spi_driver.h
*
* DESCRIPTION :
*       Header file containing the prototypes of the APIs for configuring the SPI peripheral.
*
* PUBLIC FUNCTIONS :
*       void    SPI_Init(SPI_RegDef_t* pSPI_Handle)
*       void    SPI_DeInit(SPI_RegDef_t* pSPIx)
*       void    SPI_PerClkCtrl(SPI_RegDef_t* pSPIx, uint8_t en_or_di)
*       void    SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len)
*       void    SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t len)
*       void    SPI_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       void    SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
*       void    SPI_IRQHandling(SPI_Handle_t* pHandle)
*
**/

#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include <stdint.h>
#include "stm32f446xx.h"

/**
 * Configuration structure for SPI peripheral.
 */
typedef struct
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
}SPI_Config_t;

/**
 * Handle structure for SPIx peripheral.
 */
typedef struct
{
    SPI_RegDef_t* pSPIx;            /* Base address of the SPIx peripheral */
    SPI_Config_t SPIConfig;         /* SPIx peripheral configuration settings */
}SPI_Handle_t;

/*****************************************************************************************************/
/*                                       APIs supported                                              */
/*****************************************************************************************************/

/**
 * @fn SPI_Init
 *
 * @brief function to initialize SPI peripheral.
 *
 * @param[in] pSPI_Handle handle structure for the SPI peripheral.
 *
 * @return void
 */
void SPI_Init(SPI_RegDef_t* pSPI_Handle);

/**
 *@fn SPI_DeInit
 *
 * @brief function to reset all register of a SPI peripheral.
 *
 * @param[in] pSPIx the base address of the SPIx peripheral.
 *
 * @return void
 */
void SPI_DeInit(SPI_RegDef_t* pSPIx);

/**
 * @fn SPI_PerClkCtrl
 *
 * @brief function to control the peripheral clock of the SPI peripheral.
 *
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @param[in] en_or_di for enable or disable.
 *
 * @return void
 */
void SPI_PerClkCtrl(SPI_RegDef_t* pSPIx, uint8_t en_or_di);

/**
 * @fn SPI_SendData
 *
 * @brief function to send data.
 *
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @param[in] pTxBuffer buffer with the data to send.
 * @param[in] len length of the data to send.
 *
 * @return void
 */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len);

/**
 * @fn SPI_ReceiveData
 *
 * @brief function to receive data.
 *
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @param[in] pRxBuffer buffer to store the received data.
 * @param[in] len length of the data to receive.
 *
 * @return void
 */
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t len);

/**
 * @fn SPI_IRQConfig
 *
 * @brief function to configure the IRQ number of the SPI peripheral.
 *
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] en_or_di for enable or disable.
 *
 * @return void.
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di);

/**
 * @fn SPI_IRQConfig
 *
 * @brief function to configure the IRQ number of the SPI peripheral.
 *
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] IRQPriority priority of the interrupt.
 *
 * @return void.
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @fn SPI_IRQHandling
 *
 * @brief function to handle the interrupt of the SPI peripheral.
 *
 * @param[in] pHandle handle structure to SPI peripheral.
 *
 * @return void.
 */
void SPI_IRQHandling(SPI_Handle_t* pHandle);

#endif 
