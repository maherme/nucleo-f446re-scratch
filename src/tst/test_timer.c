/********************************************************************************************************//**
* @file test_timer.c
*
* @brief File containing the APIs for testing the Timer peripheral.
*
* Public Functions:
*       - void    Timer6_Config(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include "test_timer.h"
#include "timer_driver.h"
#include "gpio_driver.h"
#include "stm32f446xx.h"

/** @brief Handler structure for Timer peripheral */
Timer_Handle_t Timer6 = {0};

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void Timer6_Config(void){

    Timer6.pTimer = TIM6;
    Timer6.prescaler = 240;
    Timer6.period = 64000 - 1;

    Timer_Init(&Timer6);
    Timer_IRQConfig(IRQ_NO_TIM6_DAC, ENABLE);
    Timer_Start(&Timer6);
}

/***********************************************************************************************************/
/*                               Weak Function Overwrite Definitions                                       */
/***********************************************************************************************************/

void TIM6_DAC_Handler(void){
    Timer_IRQHandling(&Timer6);
}

void Timer_ApplicationEventCallback(void){
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}
