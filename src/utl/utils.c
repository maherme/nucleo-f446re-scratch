/********************************************************************************************************//**
* @file utils.c
*
* @brief File containing utility functions.
*
* Public Functions:
*       - void    delay(void)
*       - void    delay_ms(uint16_t ms)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include <stdint.h>
#include "rcc_driver.h"
#include "timer_driver.h"
#include "stm32f446xx.h"

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void delay(void){
    for(uint32_t i = 0; i < 500000; i++);
}

void delay_ms(uint16_t ms){

    Timer_Handle_t Timer = {0};
    uint32_t clock = RCC_GetPCLK1Value();
    uint32_t period = ((double)ms/1000)*clock;

    Timer.tim_num = TIMER2;
    Timer.pTimer = TIM2;
    Timer.prescaler = 0;
    Timer.period = period;
    Timer_Init(&Timer);

    Timer_Start(&Timer);

    Timer.pTimer->SR &= ~(1 << TIM_SR_UIF);
    while(!(Timer.pTimer->SR & (1 << TIM_SR_UIF)));

    Timer_Stop(&Timer);
}
