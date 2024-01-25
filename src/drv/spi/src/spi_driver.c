/********************************************************************************************************//**
* @file spi_driver.c
*
* @brief File containing the APIs for configuring the SPI peripheral.
*
* Public Functions :
*       - void    SPI_Init(SPI_RegDef_t* pSPI_Handle)
*       - void    SPI_DeInit(SPI_RegDef_t* pSPIx)
*       - void    SPI_PerClkCtrl(SPI_RegDef_t* pSPIx, uint8_t en_or_di)
*       - void    SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len)
*       - void    SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t len)
*       - uint8_t SPI_SendDataIT(SPI_Handle_t* pSPI_Handle, uint8_t* pTxBuffer, uint32_t len)
*       - uint8_t SPI_ReceiveDataIT(SPI_Handle_t* pSPI_Handle, uint8_t* pRxBuffer, uint32_t len)
*       - void    SPI_IRQHandling(SPI_Handle_t* pSPI_Handle)
*       - void    SPI_Enable(SPI_RegDef_t *pSPIx, uint8_t en_or_di)
*       - void    SPI_SSICfg(SPI_RegDef_t* pSPIx, uint8_t en_or_di)
*       - void    SPI_SSOECfg(SPI_RegDef_t* pSPIx, uint8_t en_or_di)
*       - uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t flagname)
*       - void    SPI_ClearOVRFlag(SPI_RegDef_t* pSPIx)
*       - void    SPI_CloseTx(SPI_Handle_t* pSPI_Handle)
*       - void    SPI_CloseRx(SPI_Handle_t* pSPI_Handle)
*       - void    SPI_ApplicationEventCallback(SPI_Handle_t* pSPI_Handle, uint8_t app_event)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include <stdint.h>
#include <stddef.h>
#include "spi_driver.h"

/***********************************************************************************************************/
/*                                       Static Function Prototypes                                        */
/***********************************************************************************************************/

/**
 * @brief Function to handle transmission interrupt.
 * @param[in] pSPI_Handle handle structure for the SPI peripheral.
 * @return void
 */
static void spi_txe_interrupt_handle(SPI_Handle_t* pSPI_Handle);

/**
 * @brief Function to handle reception interrupt.
 * @param[in] pSPI_Handle handle structure for the SPI peripheral.
 * @return void
 */
static void spi_rxne_interrupt_handle(SPI_Handle_t* pSPI_Handle);

/**
 * @brief Function to handle error interrupt.
 * @param[in] pSPI_Handle handle structure for the SPI peripheral.
 * @return void
 */
static void spi_ovr_err_interrupt_handle(SPI_Handle_t* pSPI_Handle);

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void SPI_Init(SPI_Handle_t* pSPI_Handle){

    uint32_t temp = 0;

    /* Enable the peripheral clock */
    SPI_PerClkCtrl(pSPI_Handle->pSPIx, ENABLE);

    /* Configure the SPI_CR1 register */
    /* Configure the device mode */
    temp |= pSPI_Handle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    /* Configure the bus config */
    if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_FD){
        /* BIDI mode should be cleared */
        temp &= ~(1 << SPI_CR1_BIDI_MODE);
    }
    else if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_HD){
        /* BIDI mode should be set */
        temp |= (1 << SPI_CR1_BIDI_MODE);
    }
    else if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_S_RXONLY){
        /* BIDI mode should be cleared */
        temp &= ~(1 << SPI_CR1_BIDI_MODE);
        /* RXONLY bit must be set */
        temp |= (1 << SPI_CR1_RXONLY);
    }

    /* Configure the SPI serial clock speed (baud rate) */
    temp |= pSPI_Handle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

    /* Configure the DFF */
    temp |= pSPI_Handle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

    /* Configure the CPOL */
    temp |= pSPI_Handle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

    /* Configure the CPHA */
    temp |= pSPI_Handle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    /* Configure the SSM */
    temp |= pSPI_Handle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

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

    while(len > 0){
        /* Wait until TXE is set */
        while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
        /* Check the DFF bit in CR1 */
        if((pSPIx->CR1 & (1 << SPI_CR1_DFF))){
            /* 16 bit DFF */
            /* Load the data into the DR */
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            len--;
            len--;
            (uint16_t*)pTxBuffer++;
        }
        else{
            /* 8 bit DFF */
            pSPIx->DR = *pTxBuffer;
            len--;
            pTxBuffer++;
        }
    }
}

void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t len){

        while(len > 0){
        /* Wait until TXE is set */
        while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);
        /* Check the DFF bit in CR1 */
        if((pSPIx->CR1 & (1 << SPI_CR1_DFF))){
            /* 16 bit DFF */
            /* Load the data from DR  to RxBuffer address*/
            *((uint16_t*)pRxBuffer) = pSPIx->DR;
            len--;
            len--;
            (uint16_t*)pRxBuffer++;
        }
        else{
            /* 8 bit DFF */
            *pRxBuffer = pSPIx->DR;
            len--;
            pRxBuffer++;
        }
    }
}

uint8_t SPI_SendDataIT(SPI_Handle_t* pSPI_Handle, uint8_t* pTxBuffer, uint32_t len){

    uint8_t state = pSPI_Handle->TxState;

    if(state != SPI_BUSY_IN_TX){
        /* Save the Tx buffer address and length information */
        pSPI_Handle->pTxBuffer = pTxBuffer;
        pSPI_Handle->TxLen = len;

        /* Mark the SPI state as busy in transmission so that no other code can take over
         * same SPI peripheral until transmission is over */
        pSPI_Handle->TxState = SPI_BUSY_IN_TX;

        /* Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR */
        pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
    }

    return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t* pSPI_Handle, uint8_t* pRxBuffer, uint32_t len){

    uint8_t state = pSPI_Handle->RxState;

    if(state != SPI_BUSY_IN_RX){
        /* Save the Rx buffer address and length information */
        pSPI_Handle->pRxBuffer = pRxBuffer;
        pSPI_Handle->RxLen = len;

        /* Mark the SPI state as busy in reception so that no other code can take over
         * same SPI peripheral until reception is over */
        pSPI_Handle->RxState = SPI_BUSY_IN_RX;

        /* Enable the RXEIE control bit to get interrupt whenever RXE flag is set in SR */
        pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
    }

    return state;
}

void SPI_IRQHandling(SPI_Handle_t* pSPI_Handle){

    uint8_t temp1, temp2;

    /* Check for TXE */
    temp1 = pSPI_Handle->pSPIx->SR & (1 << SPI_SR_TXE);
    temp2 = pSPI_Handle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

    if(temp1 && temp2){
        /* Handle TXE */
        spi_txe_interrupt_handle(pSPI_Handle);
    }

    /* Check for RXNE */
    temp1 = pSPI_Handle->pSPIx->SR & (1 << SPI_SR_RXNE);
    temp2 = pSPI_Handle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

    if(temp1 && temp2){
        /* Handle RXE */
        spi_rxne_interrupt_handle(pSPI_Handle);
    }

    /* Check for OVR flag */
    temp1 = pSPI_Handle->pSPIx->SR & (1 << SPI_SR_OVR);
    temp2 = pSPI_Handle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

    if(temp1 && temp2){
        /* Handle OVR error */
        spi_ovr_err_interrupt_handle(pSPI_Handle);
    }
}

void SPI_Enable(SPI_RegDef_t *pSPIx, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else{
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

void SPI_SSICfg(SPI_RegDef_t* pSPIx, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    }
    else{
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

void SPI_SSOECfg(SPI_RegDef_t* pSPIx, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    }
    else{
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t flagname){

    if(pSPIx->SR & flagname){
        return FLAG_SET;
    }
    return FLAG_RESET;
}

void SPI_ClearOVRFlag(SPI_RegDef_t* pSPIx){

    uint8_t temp;
    temp = pSPIx->DR;
    temp = pSPIx->SR;
    (void)temp;
}

void SPI_CloseTx(SPI_Handle_t* pSPI_Handle){

    pSPI_Handle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPI_Handle->pTxBuffer = NULL;
    pSPI_Handle->TxLen = 0;
    pSPI_Handle->TxState = SPI_READY;
}

void SPI_CloseRx(SPI_Handle_t* pSPI_Handle){

    pSPI_Handle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPI_Handle->pRxBuffer = NULL;
    pSPI_Handle->RxLen = 0;
    pSPI_Handle->RxState = SPI_READY;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t* pSPI_Handle, uint8_t app_event){

    /* This is a weak implementation. The application may override this function */
    (void)pSPI_Handle;
    (void)app_event;
}

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/

static void spi_txe_interrupt_handle(SPI_Handle_t* pSPI_Handle){

    /* Check the DFF bit in CR1 */
    if((pSPI_Handle->pSPIx->CR1 & (1 << SPI_CR1_DFF))){
        /* 16 bit DFF */
        /* Load the data from DR  to RxBuffer address*/
        pSPI_Handle->pSPIx->DR = *((uint16_t*)pSPI_Handle->pTxBuffer);
        pSPI_Handle->TxLen--;
        pSPI_Handle->TxLen--;
        (uint16_t*)pSPI_Handle->pRxBuffer++;
    }
    else{
        /* 8 bit DFF */
        pSPI_Handle->pSPIx->DR = *pSPI_Handle->pTxBuffer;
        pSPI_Handle->TxLen--;
        pSPI_Handle->pRxBuffer++;
    }

    if(!pSPI_Handle->TxLen){
        /* Close SPI transmission */
        /* TX is over */
        /* Prevents interrupts from setting up of TXE flag */
        SPI_CloseTx(pSPI_Handle);
        SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_TX_CMPLT);
    }
}

static void spi_rxne_interrupt_handle(SPI_Handle_t* pSPI_Handle){

    /* Check the DFF bit in CR1 */
    if((pSPI_Handle->pSPIx->CR1 & (1 << SPI_CR1_DFF))){
        /* 16 bit DFF */
        /* Load the data from DR  to RxBuffer address*/
        *((uint16_t*)pSPI_Handle->pRxBuffer) = (uint16_t)pSPI_Handle->pSPIx->DR;
        pSPI_Handle->RxLen -= 2;
        pSPI_Handle->pRxBuffer--;
        pSPI_Handle->pRxBuffer--;
    }
    else{
        /* 8 bit DFF */
        *(pSPI_Handle->pRxBuffer) = (uint8_t)pSPI_Handle->pSPIx->DR;
        pSPI_Handle->RxLen--;
        pSPI_Handle->pRxBuffer--;
    }

    if(!pSPI_Handle->RxLen){
        /* RX is complete */
        SPI_CloseRx(pSPI_Handle);
        SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_RX_CMPLT);
    }
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t* pSPI_Handle){

    uint8_t temp;

    /* Clear the OVR flag */
    if(pSPI_Handle->TxState != SPI_BUSY_IN_TX){
        temp = pSPI_Handle->pSPIx->DR;
        temp = pSPI_Handle->pSPIx->SR;
        (void)temp;
    }

    /* Inform the application */
    SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_OVR_ERR);
}
