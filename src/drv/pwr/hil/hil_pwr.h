/********************************************************************************************************//**
* @file hil_pwr.h
*
* @brief Header file containing the prototypes of the APIs for testing the power peripheral in the hardware.
*
* Public Functions:
*       - void Test_PWR_SetPLLMax(void)
*       - void Test_SleepOnExit(void)
*       - void Test_WFE_init(void)
*       - void Test_WFE_process(void)
*       - void Test_BKRAM_init(void)
*       - void Test_BKRAM_process(void)
*       - void Test_StopMode_init(void)
*       - void Test_StopMode_irq(void)
*       - void Test_StopMode_process(void)
**/

#ifndef HIL_PWR_H
#define HIL_PWR_H

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

/**
 * @brief Function to initialize the test for the stop mode.
 * @return void.
 */
void Test_StopMode_init(void);

/**
 * @brief Function to exit from the stop mode in the test for the stop mode.
 * @return void.
 */
void Test_StopMode_irq(void);

/**
 * @brief Function to enter in the stop mode in the test for the stop mode.
 * @return void.
 */
void Test_StopMode_process(void);

#endif  /* HIL_PWR_H */
