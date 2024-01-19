/********************************************************************************************************//**
* @file hil_rtc.h
*
* @brief Header file containing the prototypes of the APIs for testing the RTC peripheral in the hardware.
*
* Public Functions:
*       - void RTC_Test_Config(void)
*       - void RTC_Test_Reset(void)
**/

#ifndef HIL_RTC_H
#define HIL_RTC_H

#if TEST_RTC

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to configure the RTC peripheral.
 * @return void
 */
void RTC_Test_Config(void);

/**
 * @brief Function to set the RTC peripheral to specified initial values.
 * @return void
 */
void RTC_Test_Reset(void);

#endif /* if TEST_RTC */

#endif /* HIL_RTC_H */
