/*****************************************************************************************************
* FILENAME :        usart_driver.h
*
* DESCRIPTION :
*       Header file containing the prototypes of the APIs for configuring the USART peripheral.
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
**/

#ifndef USART_DRIVER_H
#define USART_DRIVER_H

#include <stdint.h>
#include "stm32f446xx.h"

/**
 * Configuration structure for USARTx peripheral.
 */
typedef struct
{
    uint8_t USART_Mode;
    uint32_t USART_Baud;
    uint8_t USART_NoOfStopBits;
    uint8_t USART_WordLength;
    uint8_t USART_ParityControl;
    uint8_t USART_HWFlowControl;
}USART_Config_t;

/**
 * Handle structure for USARTx peripheral.
 */
typedef struct
{
    USART_RegDef_t* pUSARTx;
    USART_Config_t USART_Config;
}USART_Handle_t;

/*****************************************************************************************************/
/*                                       APIs Supported                                              */
/*****************************************************************************************************/

/**
 * @fn USART_Init
 *
 * @brief function to initialize USART peripheral.
 *
 * @param[in] pUSART_Handle handle structure for the USART peripheral.
 *
 * @return void
 */
void USART_Init(USART_Handle_t* pUSART_Handle);

/**
 * @fn USART_DeInit
 *
 * @brief function to reset all register of a USART peripheral.
 *
 * @param[in] pUSARTx the base address of the USARTx peripheral.
 *
 * @return void
 */
void USART_DeInit(USART_RegDef_t* pUSARTx);

/**
 * @fn USART_PerClkCtrl
 *
 * @brief function to control the peripheral clock of the USART peripheral.
 *
 * @param[in] pUSARTx the base address of the USARTx peripheral.
 * @param[in] en_or_di for enable or disable.
 *
 * @return void
 */
void USART_PerClkCtrl(USART_RegDef_t* pUSARTx, uint8_t en_or_di);

/**
 * @fn USART_SendData
 *
 * @brief function to send data.
 *
 * @param[in] pUSARTx the base address of the USARTx peripheral.
 * @param[in] pTxBuffer buffer with the data to send.
 * @param[in] len length of the data to send.
 *
 * @return void
 *
 * @note blocking call.
 */
void USART_SendData(USART_RegDef_t* pUSARTx, uint8_t* pTxBuffer, uint32_t len);

/**
 * @fn USART_ReceiveData
 *
 * @brief function to receive data.
 *
 * @param[in] pUSARTx the base address of the USARTx peripheral.
 * @param[out] pRxBuffer buffer to store the received data.
 * @param[in] len length of the data to receive.
 *
 * @return void
 *
 * @note blocking call.
 */
void USART_ReceiveData(USART_RegDef_t* pUSARTx, uint8_t* pRxBuffer, uint32_t len);

/**
 * @fn USART_SendDataIT
 *
 * @brief function to send data.
 *
 * @param[in] pUSART_Handle handle structure for the USART peripheral.
 * @param[in] pTxBuffer buffer with the data to send.
 * @param[in] len length of the data to send.
 *
 * @return @USART_APP_STATE.
 */
uint8_t USART_SendDataIT(USART_Handle_t* pUSART_Handle, uint8_t* pTxBuffer, uint32_t len);

/**
 * @fn USART_ReceiveDataIT
 *
 * @brief function to receive data.
 *
 * @param[in] pUSART_Handle handle structure for the USART peripheral.
 * @param[out] pRxBuffer buffer to store the received data.
 * @param[in] len length of the data to receive.
 *
 * @return @USART_APP_STATE.
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t* pUSART_Handle, uint8_t* pRxBuffer, uint32_t len);

/**
 * @fn USART_IRQConfig
 *
 * @brief function to configure the IRQ number of the USART peripheral.
 *
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] en_or_di for enable or disable.
 *
 * @return void.
 */
void USART_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di);

/**
 * @fn USART_IRQPriorityConfig
 *
 * @brief function to configure the IRQ number of the USART peripheral.
 *
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] IRQPriority priority of the interrupt.
 *
 * @return void.
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @fn USART_IRQHandling
 *
 * @brief function to handle the interrupt of the USART peripheral.
 *
 * @param[in] pUSART_Handle handle structure to USART peripheral.
 *
 * @return void.
 */
void USART_IRQHandling(USART_Handle_t* pUSART_Handle);

/**
 * @fn USART_Enable
 *
 * @brief function enable the USART peripheral.
 *
 * @param[in] pUSARTx the base address of the USARTx peripheral.
 * @param[in] en_or_di for enable or disable.
 *
 * @return void.
 */
void USART_Enable(USART_RegDef_t* pUSARTx, uint8_t en_or_di);

/**
 * @fn USART_GetFlagStatus
 *
 * @brief function returns the status of a given flag.
 *
 * @param[in] pUSARTx the base address of the USARTx peripheral.
 * @param[in] flagname the name of the flag.
 *
 * @return flag status: FLAG_SET or FLAG_RESET.
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTx, uint32_t flagname);

/**
 * @fn USART_ClearFlag
 *
 * @brief function clear a flag.
 *
 * @param[in] pUSARTx the base address of the USARTx peripheral.
 * @param[in] status_flagname name of the flag status.
 *
 * @return void.
 */
void USART_ClearFlag(USART_RegDef_t* pUSARTx, uint16_t status_flagname);

/**
 * @fn USART_ApplicationEventCallback
 *
 * @brief function for application callback.
 *
 * @param[in] pUSART_Handle handle structure to USART peripheral.
 * @param[in] app_event application event.
 *
 * @return void.
 */
void USART_ApplicationEventCallback(USART_Handle_t* pUSART_Handle, uint8_t app_event);

#endif /* USART_DRIVER_H */
