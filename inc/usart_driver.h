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
*       void    USART_SendData(USART_Handle_t* pUSART_Handle, uint8_t* pTxBuffer, uint32_t len)
*       void    USART_ReceiveData(USART_Handle_t* pUSART_Handle, uint8_t* pRxBuffer, uint32_t len)
*       uint8_t USART_SendDataIT(USART_Handle_t* pUSART_Handle, uint8_t* pTxBuffer, uint32_t len)
*       uint8_t USART_ReceiveDataIT(USART_Handle_t* pUSART_Handle, uint8_t* pRxBuffer, uint32_t len)
*       void    USART_SetBaudRate(USART_RegDef_t* pUSARTx, uint32_t baudrate)
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
 * @USART_DEVICE_MODE
 * USART possible device mode.
 */
#define USART_MODE_ONLY_TX      0
#define USART_MODE_ONLY_RX      1
#define USART_MODE_TXRX         2

/**
 * @USART_BAUD
 * USART possible options for baudrate.
 */
#define USART_STD_BAUD_1200     1200
#define USART_STD_BAUD_2400     2400
#define USART_STD_BAUD_9600     9600
#define USART_STD_BAUD_19200    19200
#define USART_STD_BAUD_38400    38400
#define USART_STD_BAUD_57600    57600
#define USART_STD_BAUD_115200   115200
#define USART_STD_BAUD_230400   230400
#define USART_STD_BAUD_460800   460800
#define USART_STD_BAUD_921600   921600
#define USART_STD_BAUD_2M       2000000
#define USART_STD_BAUD_3M       3000000

/**
 * @USART_PARITY_CTL
 * USART possible options for parity control.
 */
#define USART_PARITY_DISABLE    0
#define USART_PARITY_EN_EVEN    1
#define USART_PARITY_EN_ODD     2

/**
 * @USART_WORD_LENGTH
 * USART possible options for word length.
 */
#define USART_WORDLEN_8BITS     0
#define USART_WORDLEN_9BITS     1

/**
 * @USART_NUM_STOP_BITS
 * USART possible options for number of stop bits.
 */
#define USART_STOPBITS_1        0
#define USART_STOPBITS_0_5      1
#define USART_STOPBITS_2        2
#define USART_STOPBITS_1_5      3

/**
 * @USART_HW_FLOW_CTL
 * USART possible options for hardware flow control.
 */
#define USART_HW_FLOW_CTRL_NONE     0
#define USART_HW_FLOW_CTRL_CTS      1
#define USART_HW_FLOW_CTRL_RTS      2
#define USART_HW_FLOW_CTRL_CTS_RTS  3

/**
 * USART related status flags definitions.
 */
#define USART_FLAG_TXE      (1 << USART_SR_TXE)
#define USART_FLAG_TC       (1 << USART_SR_TC)
#define USART_FLAG_RXNE     (1 << USART_SR_RXNE)

/**
 * @USART_APP_STATE
 * USART possible application states.
 */
#define USART_READY         0
#define USART_BUSY_IN_RX    1
#define USART_BUSY_IN_TX    2

/**
 * USART possible application events
 */
#define USART_EVENT_TX_CMPLT    0
#define USART_EVENT_RX_CMPLT    1
#define USART_EVENT_CTS         2
#define USART_EVENT_IDLE        3
#define USART_EVENT_ORE         4
#define USART_ERROR_FE          5
#define USART_ERROR_NF          6
#define USART_ERROR_ORE         7

/**
 * Configuration structure for USARTx peripheral.
 */
typedef struct
{
    uint8_t USART_Mode;             /* Possible values from @USART_DEVICE_MODE */
    uint32_t USART_Baud;            /* Possible values from @USART_BAUD */
    uint8_t USART_NoOfStopBits;     /* Possible values from @USART_NUM_STOP_BITS */
    uint8_t USART_WordLength;       /* Possible values from @USART_WORD_LENGTH */
    uint8_t USART_ParityControl;    /* Possible values from @USART_PARITY_CTL */
    uint8_t USART_HWFlowControl;    /* Possible values from @USART_HW_FLOW_CTL */
}USART_Config_t;

/**
 * Handle structure for USARTx peripheral.
 */
typedef struct
{
    USART_RegDef_t* pUSARTx;        /* Base address of the USARTx peripheral */
    USART_Config_t USART_Config;    /* USARTx peripheral configuration settings */
    uint8_t* pTxBuffer;             /* To store the app. Tx buffer address */
    uint8_t* pRxBuffer;             /* To store the app. Rx buffer address */
    uint32_t TxLen;                 /* To store Tx len */
    uint32_t RxLen;                 /* To store Rx len */
    uint8_t TxBusyState;            /* To store busy state in transmission */
    uint8_t RxBusyState;            /* To store busy state in reception */
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
 * @param[in] pUSART_Handle handle structure for the USART peripheral.
 * @param[in] pTxBuffer buffer with the data to send.
 * @param[in] len length of the data to send.
 *
 * @return void
 *
 * @note blocking call.
 */
void USART_SendData(USART_Handle_t* pUSART_Handle, uint8_t* pTxBuffer, uint32_t len);

/**
 * @fn USART_ReceiveData
 *
 * @brief function to receive data.
 *
 * @param[in] pUSART_Handle handle structure for the USART peripheral.
 * @param[out] pRxBuffer buffer to store the received data.
 * @param[in] len length of the data to receive.
 *
 * @return void
 *
 * @note blocking call.
 */
void USART_ReceiveData(USART_Handle_t* pUSART_Handle, uint8_t* pRxBuffer, uint32_t len);

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
 * @fn USART_SetBaudRate
 *
 * @brief function to set the baud rate of the USART peripheral.
 *
 * @param[in] pUSART_Handle handle structure for the USART peripheral.
 * @param[in] baudrate possible values of baud rate.
 *
 * @return void
 */
void USART_SetBaudRate(USART_RegDef_t* pUSARTx, uint32_t baudrate);

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
