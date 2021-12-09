/********************************************************************************************************//**
* @file test_rcc.h
*
* @brief Header file containing the prototypes of the APIs for testing the RCC peripheral.
*
* Public Functions:
*       - void SetHSEBypass(void)
*       - void SetPLLMax(void)
*       - void SetMCO_LSE_HSE(void)
*       - void SetMCO_PLL(void)
**/

#ifndef TEST_RCC_H
#define TEST_RCC_H

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function for setting the HSE clock source in bypass mode.
 * @return void.
 */
void SetHSEBypass(void);

/**
 * @brief Function for setting the PLL at maximum clock frequency with HSE as input source.
 * @return void.
 */
void SetPLLMax(void);

/**
 * @brief Function for setting the MCO1 and MCO2.
 * @return void
 */
void SetMCO_LSE_HSE(void);

/**
 * @brief Function for settin the MCO1.
 * @return void
 */
void SetMCO_PLL(void);

#endif /* TEST_RCC_H */
