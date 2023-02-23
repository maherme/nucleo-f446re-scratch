/********************************************************************************************************//**
* @file test_usart.h
*
* @brief Header file containing the prototypes of the APIs for testing the USART peripheral.
*
* Public Functions:
*       - void    USART3_Config(void)
*       - void    USART3_SendHello(void)
*       - void    USART3_TxRx(void)
**/

#ifndef TEST_USART_H
#define TEST_USART_H

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to initialize and configure USART3 peripheral.
 * @return void.
 */
void USART3_Config(void);

/**
 * @brief Function to test USART3 peripheral transmission.
 * @return void.
 */
void USART3_SendHello(void);

/**
 * @brief Function to test USART3 peripheral transmission and reception.
 * @return void.
 */
void USART3_TxRx(void);

#endif /* TEST_USART_H */
