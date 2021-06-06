/*****************************************************************************************************
* FILENAME :        spi_driver.h
*
* DESCRIPTION :
*       Header file containing the prototypes of the APIs for configuring the SPI peripheral.
*
* PUBLIC FUNCTIONS :
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
}GPIO_Handle_t;

#endif 
