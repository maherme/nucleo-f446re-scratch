/********************************************************************************************************//**
* @file test.h
*
* @brief Header file containing prototypes of functions for testing drivers.
*
* PUBLIC FUNCTIONS :
*       void    test_init(void)
*       void    test_process(void)
**/

#ifndef TEST_H
#define TEST_H

/**
 * @name Flags for enabling peripheral testing.
 * @{
 */
#define TEST_SPI    0   /**< @brief Set to 1 for enabling the SPI test */
#define TEST_I2C    0   /**< @brief Set to 1 for enabling the I2C test */
#define TEST_USART  0   /**< @brief Set to 1 for enabling the USART test */
#define TEST_RCC    0   /**< @brief Set to 1 for enabling the RCC test */
#define TEST_TIMER  0   /**< @brief Set to 1 for enabling the Timer test */
#define TEST_DMA    0   /**< @brief Set to 1 for enabling the DMA test */
#define TEST_RTC    1   /**< @brief Set to 1 for enabling the RTC test */
/** @} */

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to initialize and configure hardware needed for testing.
 * @return void.
 */
void test_init(void);

/**
 * @brief Function to be executed as process in the main loop.
 * @return void.
 */
void test_process(void);

#endif /* TEST_H */
