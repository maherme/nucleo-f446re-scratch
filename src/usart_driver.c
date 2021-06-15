/*****************************************************************************************************
* FILENAME :        usart_driver.c
*
* DESCRIPTION :
*       File containing the APIs for configuring the USART peripheral.
*
* PUBLIC FUNCTIONS :
*       void    USART_Init(USART_Handle_t* pUSART_Handle)
*       void    USART_DeInit(USART_RegDef_t* pUSARTx)
*       void    USART_PerClkCtrl(USART_RegDef_t* pUSARTx, uint8_t en_or_di)
*       void    USART_SendData(USART_Handle_t* pUSART_Handle, uint8_t* pTxBuffer, uint32_t len)
*       void    USART_ReceiveData(USART_Handle_t* pUSART_Handle, uint8_t* pRxBuffer, uint32_t len)
*       uint8_t USART_SendDataIT(USART_Handle_t* pUSART_Handle, uint8_t* pTxBuffer, uint32_t len)
*       uint8_t USART_ReceiveDataIT(USART_Handle_t* pUSART_Handle, uint8_t* pRxBuffer, uint32_t len)
*       void    USART_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       void    USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
*       void    USART_IRQHandling(USART_Handle_t* pUSART_Handle)
*       void    USART_Enable(USART_RegDef_t* pUSARTx, uint8_t en_or_di)
*       uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTx, uint32_t flagname)
*       void    USART_ClearFlag(USART_RegDef_t* pUSARTx, uint16_t status_flagname)
*       void    USART_ApplicationEventCallback(USART_Handle_t* pUSART_Handle, uint8_t app_event)
*
* NOTES :
*       For further information about functions refer to the corresponding header file.
*
**/

#include <stdint.h>
#include "usart_driver.h"

/*****************************************************************************************************/
/*                                       Public API Definitions                                      */
/*****************************************************************************************************/

void USART_Init(USART_Handle_t* pUSART_Handle){

    uint32_t temp = 0;

    /* Enable the peripheral clock */
    USART_PerClkCtrl(pUSART_Handle->pUSARTx, ENABLE);

    /* Enable USART TX and RX engines */
    if(pUSART_Handle->USART_Config.USART_Mode == USART_MODE_ONLY_RX){
        /* Enable receiver bit field */
        temp |= (1 << USART_CR1_RE);
    }
    else if(pUSART_Handle->USART_Config.USART_Mode == USART_MODE_ONLY_TX){
        /* Enable transmitter bit field */
        temp |= (1 << USART_CR1_TE);
    }
    else if(pUSART_Handle->USART_Config.USART_Mode == USART_MODE_TXRX){
        /* Enable both transmitter and receiver bit field */
        temp |= ((1 << USART_CR1_TE) | (1 << USART_CR1_RE));
    }

    /* Configure the word length */
    temp |= pUSART_Handle->USART_Config.USART_WordLength << USART_CR1_M;

    /* Configure parity control bit field */
    if(pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN){
        /* Enable parity control */
        /* EVEN parity set by default, so no need to implement */
        temp |= (1 << USART_CR1_PCE);
    }
    else if(pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD){
        /* Enable parity control */
        temp |= (1 << USART_CR1_PCE);

        /* Enable ODD parity */
        temp |= (1 << USART_CR1_PS);
    }

    /* Program CR1 register */
    pUSART_Handle->pUSARTx->CR1 = temp;
}

void USART_DeInit(USART_RegDef_t* pUSARTx){

    if(pUSARTx == USART1){
        USART1_REG_RESET();
    }
    else if(pUSARTx == USART2){
        USART2_REG_RESET();
    }
    else if(pUSARTx == USART3){
        USART3_REG_RESET();
    }
    else if(pUSARTx == UART4){
        UART4_REG_RESET();
    }
    else if(pUSARTx == UART5){
        UART5_REG_RESET();
    }
    else if(pUSARTx == USART6){
        USART6_REG_RESET();
    }
    else{
        /* do nothing */
    }
}

void USART_PerClkCtrl(USART_RegDef_t* pUSARTx, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        if(pUSARTx == USART1){
            USART1_PCLK_EN();
        }
        else if(pUSARTx == USART2){
            USART2_PCLK_EN();
        }
        else if(pUSARTx == USART3){
            USART3_PCLK_EN();
        }
        else if(pUSARTx == UART4){
            UART4_PCLK_EN();
        }
        else if(pUSARTx == UART5){
            UART5_PCLK_EN();
        }
        else if(pUSARTx == USART6){
            USART6_PCLK_EN();
        }
        else{
            /* do nothing */
        }
    }
    else{
        if(pUSARTx == USART1){
            USART1_PCLK_DI();
        }
        else if(pUSARTx == USART2){
            USART2_PCLK_DI();
        }
        else if(pUSARTx == USART3){
            USART3_PCLK_DI();
        }
        else if(pUSARTx == UART4){
            UART4_PCLK_DI();
        }
        else if(pUSARTx == UART5){
            UART5_PCLK_DI();
        }
        else if(pUSARTx == USART6){
            USART6_PCLK_DI();
        }
        else{
            /* do nothing */
        }
    }
}

void USART_SendData(USART_Handle_t* pUSART_Handle, uint8_t* pTxBuffer, uint32_t len){

    uint32_t i;
    uint16_t* pdata;

    /* Loop for transmitting len bytes */
    for(i = 0; i < len; i++){
        /* Wait until TXE flag is set in SR */
        while(!USART_GetFlagStatus(pUSART_Handle->pUSARTx, USART_FLAG_TXE));

        /* Check USART word length for 9 bits or 8 bits in a frame */
        if(pUSART_Handle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
            /* If 9 bits load the DR with 2 bytes masking the bits other than first 9 bits */
            pdata = (uint16_t*)pTxBuffer;
            pUSART_Handle->pUSARTx->DR = (*pdata & (uint16_t)0x1FF);

            /* Check for USART parity control */
            if(pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
                /* 9 bits of user data will be sent */
                pTxBuffer++;
                pTxBuffer++;
            }
            else{
                /* 8 bits of user data will be sent */
                /* 9th bit will be replaced by parity bit by hardware */
                pTxBuffer++;
            }
        }
        else{
            /* 8 bits data transfer */
            pUSART_Handle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);

            /* Increment buffer adddress */
            pTxBuffer++;
        }
    }

    /* Wait until TC flag is set in SR */
    while(!USART_GetFlagStatus(pUSART_Handle->pUSARTx, USART_FLAG_TC));
}

void USART_ReceiveData(USART_Handle_t* pUSART_Handle, uint8_t* pRxBuffer, uint32_t len){

    uint32_t i;

    for(i = 0; i < len; i++){
        /* Wait until RXNE flag is set in SR */
        while(!USART_GetFlagStatus(pUSART_Handle->pUSARTx, USART_FLAG_RXNE));

        /* Check USART word length for 9 bits or 8 bits in a frame */
        if(pUSART_Handle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
            /* Reception of 9 bits data frame */
            /* Check for USART parity control */
            if(pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
                /* 9 bits are user data */
                *((uint16_t*)pRxBuffer) = (pUSART_Handle->pUSARTx->DR & (uint16_t)0x01FF);
                /* Increment the pRxBuffer two times, once per byte */
                pRxBuffer++;
                pRxBuffer++;
            }
            else{
                /* 8 bits are user data and 1 bit is parity */
                *pRxBuffer = (pUSART_Handle->pUSARTx->DR & (uint8_t)0xFF);
                pRxBuffer++;
            }
        }
        else{
            /* Reception of 8 bits data frame */
            /* Check for USART parity control */
            if(pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
                /* 8 bits are user data */
                *pRxBuffer = (uint8_t)(pUSART_Handle->pUSARTx->DR & (uint8_t)0xFF);
            }
            else{
                /* 7 bits are user data and 1 bit is parity */
                *pRxBuffer = (uint8_t)(pUSART_Handle->pUSARTx->DR & (uint8_t)0x7F);
            }
            pRxBuffer++;
        }
    }
}

uint8_t USART_SendDataIT(USART_Handle_t* pUSART_Handle, uint8_t* pTxBuffer, uint32_t len){

    uint8_t txstate = pUSART_Handle->TxBusyState;

    if(txstate != USART_BUSY_IN_TX){
        pUSART_Handle->TxLen = len;
        pUSART_Handle->pTxBuffer = pTxBuffer;
        pUSART_Handle->TxBusyState = USART_BUSY_IN_TX;

        /* Enable interrupt for TXE */
        pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

        /* Enable interrupt for TC */
        pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
    }

    return txstate;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t* pUSART_Handle, uint8_t* pRxBuffer, uint32_t len){

    uint8_t rxstate = pUSART_Handle->RxBusyState;

    if(rxstate != USART_BUSY_IN_RX){
        pUSART_Handle->RxLen = len;
        pUSART_Handle->pRxBuffer = pRxBuffer;
        pUSART_Handle->RxBusyState = USART_BUSY_IN_RX;

        /* Enable interrupt for RXNE */
        pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
    }

    return rxstate;
}

void USART_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        if(IRQNumber <= 31){
            /* Program ISER0 register */
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64){
            /* Program ISER1 register */
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96){
            /* Program ISER2 register */
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
        else{
            /* do nothing */
        }
    }
    else{
        if(IRQNumber <= 31){
            /* Program ICER0 register */
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64){
            /* Program ICER1 register */
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96){
            /* Program ICER2 register */
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
        else{
            /* do nothing */
        }
    }
}

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
    /* Find out the IPR register */
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift = (8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift);
}

void USART_IRQHandling(USART_Handle_t* pUSART_Handle){
}

void USART_Enable(USART_RegDef_t* pUSARTx, uint8_t en_or_di){
}

uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTx, uint32_t flagname){

    if(pUSARTx->SR & flagname){
        return SET;
    }

    return RESET;
}

void USART_ClearFlag(USART_RegDef_t* pUSARTx, uint16_t status_flagname){
}

__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t* pUSART_Handle, uint8_t app_event){

    /* This is a weak implementation. The application may override this function */
}
