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
    /* Configure the SPI_CR1 register */
    uint32_t temp = 0;

    /* Configure the device mode */
    temp |= pSPI_Handle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    /* Configure the bus config */
    if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_FD){
        /* BIDI mode should be cleared */
        temp &= ~(1 << 15);
    }
    else if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_HD){
        /* BIDI mode should be set */
        temp |= (1 << 15);
    }
    else if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_S_RXONLY){
        /* BIDI mode should be cleared */
        temp &= ~(1 << 15);
        /* RXONLY bit must be set */
        temp |= (1 << 10);
    }

    /* Configure the SPI serial clock speed (baud rate) */
    temp |= pSPI_Handle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

    /* Configure the DFF */
    temp |= pSPI_Handle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

    /* Configure the CPOL */
    temp |= pSPI_Handle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

    /* Configure the CPHA */
    temp |= pSPI_Handle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    pSPI_Handle->pSPIx->CR1 = temp;
}

void SPI_DeInit(SPI_RegDef_t* pSPIx){

    if(pSPIx == SPI1){
        SPI1_REG_RESET();
    }
    else if(pSPIx == SPI2){
        SPI2_REG_RESET();
    }
    else if(pSPIx == SPI3){
        SPI3_REG_RESET();
    }
    else if(pSPIx == SPI4){
        SPI4_REG_RESET();
    }
    else{
        /* do nothing */
    }
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
