/*****************************************************************************************************
* FILENAME :        usart_driver.h
*
* DESCRIPTION :
*       Header file containing the prototypes of the APIs for configuring the USART peripheral.
*
* PUBLIC FUNCTIONS :
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
    uint8_t USART NoOfStopBits;
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

#endif /* USART_DRIVER_H */
