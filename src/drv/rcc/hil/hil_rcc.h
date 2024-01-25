/********************************************************************************************************//**
* @file hil_rcc.h
*
* @brief Header file containing the prototypes of the APIs for testing the RCC peripheral in the hardware.
*
* Public Functions:
*       - void SetHSEBypass(void)
*       - void SetPLLMax(void)
*       - void SetMCO_LSE_HSE(void)
*       - void SetMCO_PLL(void)
**/

#ifndef HIL_RCC_H
#define HIL_RCC_H

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

#endif /* HIL_RCC_H */
