/********************************************************************************************************//**
* @file test_pwr.h
*
* @brief Header file containing the prototypes of the APIs for testing the power peripheral.
*
* Public Functions:
*       - void Test_PWR_SetPLLMax(void)
*       - void Test_SleepOnExit(void)
*       - void Test_WFE_init(void)
*       - void Test_WFE_process(void)
*       - void Test_BKRAM_init(void)
*       - void Test_BKRAM_process(void)
**/

#ifndef TEST_PWR_H
#define TEST_PWR_H

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to test the PLL to maximum frequency.
 * @return void.
 */
void Test_PWR_SetPLLMax(void);

/**
 * @brief Function to test the sleep on exit core functionality.
 * @return void.
 */
void Test_SleepOnExit(void);

/**
 * @brief Function to initialize the test for the WFE instruction.
 * @return void.
 */
void Test_WFE_init(void);

/**
 * @brief Function to test the WFE instruction in the main loop. Pressing the button connected to the PC13
 *        the CPU exits from low power state.
 * @return void.
 */
void Test_WFE_process(void);

/**
 * @brief Function to initialize the test for the backup SRAM interface.
 * @return void.
 */
void Test_BKRAM_init(void);

/**
 * @brief Function to enter in standby mode for testing the backup SRAM interface.
 * @return void.
 */
void Test_BKRAM_process(void);

#endif  /* TEST_PWR_H */