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
*       void    USART_SendData(USART_RegDef_t* pUSARTx, uint8_t* pTxBuffer, uint32_t len)
*       void    USART_ReceiveData(USART_RegDef_t* pUSARTx, uint8_t* pRxBuffer, uint32_t len)
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
}

void USART_DeInit(USART_RegDef_t* pUSARTx){
}

void USART_PerClkCtrl(USART_RegDef_t* pUSARTx, uint8_t en_or_di){
}

void USART_SendData(USART_RegDef_t* pUSARTx, uint8_t* pTxBuffer, uint32_t len){
}

void USART_ReceiveData(USART_RegDef_t* pUSARTx, uint8_t* pRxBuffer, uint32_t len){
}

uint8_t USART_SendDataIT(USART_Handle_t* pUSART_Handle, uint8_t* pTxBuffer, uint32_t len){
    return 0;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t* pUSART_Handle, uint8_t* pRxBuffer, uint32_t len){
    return 0;
}

void USART_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di){
}

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
}

void USART_IRQHandling(USART_Handle_t* pUSART_Handle){
}

void USART_Enable(USART_RegDef_t* pUSARTx, uint8_t en_or_di){
}

uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTx, uint32_t flagname){
    return 0;
}

void USART_ClearFlag(USART_RegDef_t* pUSARTx, uint16_t status_flagname){
}

__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t* pUSART_Handle, uint8_t app_event){

    /* This is a weak implementation. The application may override this function */
}
