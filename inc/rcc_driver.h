/*****************************************************************************************************
* FILENAME :        rcc_driver.h
*
* DESCRIPTION :
*       Header file containing the prototypes of the APIs for configuring the RCC peripheral.
*
* PUBLIC FUNCTIONS :
*       uint32_t    RCC_GetPCLK1Value(void)
*       uint32_t    RCC_GetPCLK2Value(void)
*       uint32_t    RCC_GetPLLOutputClock(void)
*
**/

#ifndef RCC_DRIVER_H
#define RCC_DRIVER_H

#include <stdint.h>

/*****************************************************************************************************/
/*                                       APIs Supported                                              */
/*****************************************************************************************************/

/**
 * @fn RCC_GetPCLK1Value
 *
 * @brief function to calculate the PCLK1 clock value.
 *
 * @return PCLK1 clock value.
 */
uint32_t RCC_GetPCLK1Value(void);

/**
 * @fn RCC_GetPCLK1Value
 *
 * @brief function to calculate the PCLK2 clock value.
 *
 * @return PCLK1 clock value.
 */
uint32_t RCC_GetPCLK2Value(void);

/**
 * @fn RCC_GetPLLOutputClock
 *
 * @brief function to calculate the PLL clock value.
 *
 * @return PLL clock value.
 */
uint32_t RCC_GetPLLOutputClock(void);

#endif /* RCC_DRIVER_H */
