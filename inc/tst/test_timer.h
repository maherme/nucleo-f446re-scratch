/********************************************************************************************************//**
* @file test_timer.h
*
* @brief Header file containing the prototypes of the APIs for testing the Timer peripheral.
*
* Public Functions:
*       - void    Timer6_Config(void)
*       - void    Timer2_Config(void)
*       - void    Timer2_Process(void)
**/

#ifndef TEST_TIMER_H
#define TEST_TIMER_H

/**
 * @brief Function to initialize and configure TIM6 peripheral.
 * @return void.
 */
void Timer6_Config(void);

/**
 * @brief Function to initialize and configure TIM2 peripheral for testing the input capture functionality.
 *        This test consit on generating a signal using the LSE (32.768KHz) and MCO1 (output pin PA8), and
 *        capture this signal using the capture/compare channel 1 of TIM2 peripheral (input pin PA0).
 * @return void.
 */
void Timer2_Config(void);

/**
 * @brief Function to perform the TIM2 test in main loop.
 * @return void.
 */
void Timer2_Process(void);

#endif /* TEST_TIMER_H */
