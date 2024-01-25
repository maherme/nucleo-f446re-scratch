/********************************************************************************************************//**
* @file hil.h
*
* @brief Header file containing prototypes of functions for testing drivers in the hardware.
*
* PUBLIC FUNCTIONS :
*       void    hil_init(void)
*       void    hil_process(void)
**/

#ifndef HIL_H
#define HIL_H

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
#define TEST_RTC    0   /**< @brief Set to 1 for enabling the RTC test */
#define TEST_CAN    0   /**< @brief Set to 1 for enabling the CAN test */
#define TEST_PWR    0   /**< @brief Set to 1 for enabling the PWR test */
/** @} */

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to initialize and configure hardware needed for testing.
 * @return void.
 */
void hil_init(void);

/**
 * @brief Function to be executed as process in the main loop.
 * @return void.
 */
void hil_process(void);

#endif /* HIL_H */
