/********************************************************************************************************//**
* @file utils.h
*
* @brief Header file containing the prototypes of the utility functions.
*
* Public Functions:
*       - void    delay(void)
*       - void    delay_ms(uint16_t ms)
**/

#ifndef UTILS_H
#define UTILS_H

/**
 * @brief Function to insert some delay in the execution
 * @return void.
 */
void delay(void);

/**
 * @brief Function to insert a delay in milliseconds.
 * @param[in] ms is the number of milliseconds for the delay.
 * @return void.
 * @note This function uses the TIM2 peripheral.
 */
void delay_ms(uint16_t ms);

#endif
