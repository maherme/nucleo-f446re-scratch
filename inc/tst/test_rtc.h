/********************************************************************************************************//**
* @file test_rtc.h
*
* @brief Header file containing the prototypes of the APIs for testing the RTC peripheral.
*
* Public Functions:
*       - void RTC_Test_Config(void)
*       - void RTC_Test_Reset(void)
**/

#ifndef TEST_RTC_H
#define TEST_RTC_H

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

#endif /* TEST_RTC_H */
