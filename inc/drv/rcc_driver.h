/********************************************************************************************************//**
* @file rcc_driver.h
*
* @brief Header file containing the prototypes of the APIs for configuring the RCC peripheral.
*
* Public Functions:
*       - uint32_t RCC_GetPCLK1Value(void)
*       - uint32_t RCC_GetPCLK2Value(void)
*       - uint32_t RCC_GetPLLOutputClock(void)
*/

#ifndef RCC_DRIVER_H
#define RCC_DRIVER_H

#include <stdint.h>

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to calculate the PCLK1 clock value.
 * @return PCLK1 clock value.
 */
uint32_t RCC_GetPCLK1Value(void);

/**
 * @brief Function to calculate the PCLK2 clock value.
 * @return PCLK1 clock value.
 */
uint32_t RCC_GetPCLK2Value(void);

/**
 * @brief Function to calculate the PLL clock value.
 * @return PLL clock value.
 */
uint32_t RCC_GetPLLOutputClock(void);

#endif /* RCC_DRIVER_H */
