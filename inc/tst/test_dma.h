/********************************************************************************************************//**
* @file test_dma.h
*
* @brief Header file containing the prototypes of the APIs for testing the DMA peripheral.
*
* Public Functions:
*       - void      DMA1_Config(void)
*       - void      DMA1_USART3_Request(void)
*       - void      DMA1_USART3_Request_IT(void)
**/

#ifndef TEST_DMA_H
#define TEST_DMA_H

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to initialize and configure DMA1 peripheral.
 * @return void.
 */
void DMA1_Config(void);

/**
 * @brief Function for sending a request to DMA1 from USART3 peripheral.
 * @return void
 *
 * @note Use this API when DMA interrupts are not configured.
 */
void DMA1_USART3_Request(void);

/**
 * @brief Function for sending a request to DMA1 from USART3 peripheral.
 * @return void
 *
 * @note Use this API when DMA interrupts are configured.
 */
void DMA1_USART3_Request_IT(void);

#endif /* TEST_DMA_H */
