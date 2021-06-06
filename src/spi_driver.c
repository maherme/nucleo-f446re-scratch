/*****************************************************************************************************
* FILENAME :        spi_driver.c
*
* DESCRIPTION :
*       File containing the APIs for configuring the SPI peripheral.
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
* NOTES :
*       For further information about functions refer to the corresponding header file.
*
**/

#include <stdint.h>
#include "spi_driver.h"

void SPI_Init(SPI_RegDef_t* pSPI_Handle){
}

void SPI_DeInit(SPI_RegDef_t* pSPIx){
}

void SPI_PerClkCtrl(SPI_RegDef_t* pSPIx, uint8_t en_or_di){
}

void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len){
}

void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t len){
}

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di){
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
}

void SPI_IRQHandling(SPI_Handle_t* pHandle){
}
