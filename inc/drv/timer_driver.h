/********************************************************************************************************//**
* @file timer_driver.h
*
* @brief Header file containing the prototypes of the APIs for configuring the TIM peripheral.
*
* Public Functions:
*       - void    Timer_Init(Timer_Handle_t* Timer_Handle)
*       - void    Timer_Start(Timer_Handle_t* Timer_Handle)
*       - void    Timer_PerClkCtrl(TIM6_7_RegDef_t* pTimer, uint8_t en_or_di)
*       - void    Timer_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       - void    Timer_IRQHandling(Timer_Handle_t* Timer_Handle)
*       - void    Timer_ApplicationEventCallback(void)
*/

#ifndef TIMER_DRIVER_H
#define TIMER_DRIVER_H

#include <stdint.h>
#include "stm32f446xx.h"

/**
 * @brief Handle structure for timer peripheral.
 */
typedef struct
{
    TIM6_7_RegDef_t* pTimer;    /**< Base address of the timer peripheral */
    uint16_t prescaler;         /**< Prescaler value */
    uint16_t period;            /**< Period value */
}Timer_Handle_t;


/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to initialize the timer peripheral.
 * @param[in] Timer_Handle handle structure for managing the timer peripheral.
 * @return void
 */
void Timer_Init(Timer_Handle_t* Timer_Handle);

/**
 * @brief Function to start working the timer peripheral.
 * @param[in] Timer_Handle handle structure for managing the timer peripheral.
 * @return void
 */
void Timer_Start(Timer_Handle_t* Timer_Handle);

/**
 * @brief Function to control the peripheral clock of the timer peripheral.
 * @param[in] pTimer the base address of the TIM6 or TIM7 peripheral.
 * @param[in] en_or_di for enable or disable.
 * @return void
 */
void Timer_PerClkCtrl(TIM6_7_RegDef_t* pTimer, uint8_t en_or_di);

/**
 * @brief Function to configure the IRQ number of the timer peripheral.
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] en_or_di for enable or disable.
 * @return void.
 */
void Timer_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di);

/**
 * @brief Function to handle the interrupt of the timer peripheral.
 * @param[in] Timer_Handle handle structure to timer peripheral.
 * @return void.
 */
void Timer_IRQHandling(Timer_Handle_t* Timer_Handle);

/**
 * @brief Function for application callback.
 * @return void.
 */
void Timer_ApplicationEventCallback(void);

#endif /* TIMER_DRIVER_H */
