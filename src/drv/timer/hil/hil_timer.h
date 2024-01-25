/********************************************************************************************************//**
* @file hil_timer.h
*
* @brief Header file containing the prototypes of the APIs for testing the Timer peripheral in the hardware.
*
* Public Functions:
*       - void    Timer6_Config(void)
*       - void    Timer2_Config(void)
*       - void    Timer2_Process(void)
*       - void    Timer4_Config(void)
*       - void    Timer3_Config(void)
*       - void    Timer3_Process(void)
**/

#ifndef HIL_TIMER_H
#define HIL_TIMER_H

/**
 * @brief Function to initialize and configure TIM6 peripheral.
 * @return void.
 */
void Timer6_Config(void);

/**
 * @brief Function to initialize and configure TIM2 peripheral for testing the input capture functionality.
 *        This test consist on generating a signal using the LSE (32.768KHz) and MCO1 (output pin PA8), and
 *        capture this signal using the capture/compare channel 1 of TIM2 peripheral (input pin PA0).
 * @return void.
 */
void Timer2_Config(void);

/**
 * @brief Function to perform the TIM2 test in main loop.
 * @return void.
 */
void Timer2_Process(void);

/**
 * @brief Function to initialize and configure TIM4 peripheral for testing the output capture functionality.
 *        This test consist on generating 4 signals of 80Hz, 160Hz, 320Hz and 640Hz using the 4 channel of
 *        the TIM4 peripheral.
 * @return void.
 */
void Timer4_Config(void);

/**
 * @brief Function to initialize and configure TIM3 peripheral as PWM and connect it to the pin PA6 for
 *        generating a PWM signal.
 * @return void.
 */
void Timer3_Config(void);

/**
 * @brief Function to perform the TIM3 test in main loop. This test consist on generating a variable PWM
 *        signal which powers an external LED for controlling the brightness.
 * @return void.
 */
void Timer3_Process(void);

#endif /* HIL_TIMER_H */
