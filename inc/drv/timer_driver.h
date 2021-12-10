/********************************************************************************************************//**
* @file timer_driver.h
*
* @brief Header file containing the prototypes of the APIs for configuring the TIM peripheral.
*
* Public Functions:
*       - void    Timer_Init(Timer_Handle_t* Timer_Handle)
*       - void    Timer_Start(Timer_Handle_t* Timer_Handle)
*       - void    Timer_PerClkCtrl(Timer_Num_t timer_num, uint8_t en_or_di)
*       - void    Timer_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       - void    Timer_IRQHandling(Timer_Handle_t* Timer_Handle)
*       - void    Timer_ApplicationEventCallback(void)
*/

#ifndef TIMER_DRIVER_H
#define TIMER_DRIVER_H

#include <stdint.h>
#include "stm32f446xx.h"

/**
 * @brief Enum for selecting timer peripheral.
 */
typedef enum
{
    TIMER1,     /**< TIM1 Advanced-control timer */
    TIMER2,     /**< TIM2 General-purpose timer */
    TIMER3,     /**< TIM3 General-purpose timer */
    TIMER4,     /**< TIM4 General-purpose timer */
    TIMER5,     /**< TIM5 General-purpose timer */
    TIMER6,     /**< TIM6 Basic timer */
    TIMER7,     /**< TIM7 Basic timer */
    TIMER8,     /**< TIM8 Advanced-control timer */
    TIMER9,     /**< TIM9 General-purpose timer */
    TIMER10,    /**< TIM10 General-purpose timer */
    TIMER11,    /**< TIM11 General-purpose timer */
    TIMER12,    /**< TIM12 General-purpose timer */
    TIMER13,    /**< TIM13 General-purpose timer */
    TIMER14     /**< TIM14 General purpose timer */
}Timer_Num_t;

/**
 * @brief Handle structure for timer peripheral.
 */
typedef struct
{
    Timer_Num_t tim_num;        /**< TIMx peripheral */
    TIM_RegDef_t* pTimer;       /**< Base address of the timer peripheral */
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
 * @param[in] timer_num is the TIM peripheral number.
 * @param[in] en_or_di for enable or disable.
 * @return void
 */
void Timer_PerClkCtrl(Timer_Num_t timer_num, uint8_t en_or_di);

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
