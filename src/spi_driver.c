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

void SPI_Init(SPI_Handle_t* pSPI_Handle){
}

void SPI_DeInit(SPI_RegDef_t* pSPIx){
}

void SPI_PerClkCtrl(SPI_RegDef_t* pSPIx, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        if(pSPIx == SPI1){
            SPI1_PCLK_EN();
        }
        else if(pSPIx == SPI2){
            SPI2_PCLK_EN();
        }
        else if(pSPIx == SPI3){
            SPI3_PCLK_EN();
        }
        else if(pSPIx == SPI4){
            SPI4_PCLK_EN();
        }
        else{
            /* do nothing */
        }
    }
    else{
        if(pSPIx == SPI1){
            SPI1_PCLK_DI();
        }
        else if(pSPIx == SPI2){
            SPI2_PCLK_DI();
        }
        else if(pSPIx == SPI3){
            SPI3_PCLK_DI();
        }
        else if(pSPIx == SPI4){
            SPI4_PCLK_DI();
        }
        else{
            /* do nothing */
        }
    }
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
