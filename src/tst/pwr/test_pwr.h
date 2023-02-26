/********************************************************************************************************//**
* @file test_pwr.h
*
* @brief Header file containing the prototypes of the APIs for testing the power peripheral.
*
* Public Functions:
*       - void Test_PWR_SetPLLMax(void)
*       - void Test_SleepOnExit(void)
**/

#ifndef TEST_PWR_H
#define TEST_PWR_H

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to set the PLL to maximum frequency.
 * @return void.
 */
void Test_PWR_SetPLLMax(void);

/**
 * @brief Function to set the sleep on exit core functionality.
 * @return void.
 */
void Test_SleepOnExit(void);

#endif  /* TEST_PWR_H */