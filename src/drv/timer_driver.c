/********************************************************************************************************//**
* @file timer_driver.c
*
* @brief File containing the APIs for configuring the TIM peripheral.
*
* Public Functions:
*       - void Timer_Init(Timer_Handle_t* Timer_Handle)
*       - void Timer_Start(Timer_Handle_t* Timer_Handle)
*       - void Timer_PerClkCtrl(TIM6_7_RegDef_t* pTimer, uint8_t en_or_di)
*       - void Timer_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       - void Timer_IRQHandling(Timer_Handle_t* Timer_Handle)
*       - void Timer_ApplicationEventCallback(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
*/

#include <stdint.h>
#include "timer_driver.h"
#include "stm32f446xx.h"

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void Timer_Init(Timer_Handle_t* Timer_Handle){

    /* Enable peripheral clock */
    Timer_PerClkCtrl(Timer_Handle->pTimer, ENABLE);
    /* Clear and set prescaler value */
    Timer_Handle->pTimer->PSC &= ~(0xFFFF << TIM6_7_PSC);
    Timer_Handle->pTimer->PSC |= (Timer_Handle->prescaler << TIM6_7_PSC);
    /*Clear and set period value */
    Timer_Handle->pTimer->ARR &= ~(0xFFFF << TIM6_7_ARR);
    Timer_Handle->pTimer->ARR |= (Timer_Handle->period << TIM6_7_ARR);
    /* Enable interrupt */
    Timer_Handle->pTimer->DIER |= (1 << TIM6_7_DIER_UIE);
}

void Timer_Start(Timer_Handle_t* Timer_Handle){

    /* Clear UIF bit in SR register */
    Timer_Handle->pTimer->SR &= ~(1 << TIM6_7_SR_UIF);
    /* Set counter enable */
    Timer_Handle->pTimer->CR1 |= (1 << TIM6_7_CR1_CEN);
}

void Timer_PerClkCtrl(TIM6_7_RegDef_t* pTimer, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        if(pTimer == TIM6){
            TIM6_PCLK_EN();
        }
        else if(pTimer == TIM7){
            TIM7_PCLK_EN();
        }
        else{
            /* do nothing */
        }
    }
    else{
        if(pTimer == TIM6){
            TIM6_PCLK_DI();
        }
        else if(pTimer == TIM7){
            TIM7_PCLK_DI();
        }
        else{
            /* do nothing */
        }
    }
}

void Timer_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        if(IRQNumber <= 31){
            /* Program ISER0 register */
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64){
            /* Program ISER1 register */
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96){
            /* Program ISER2 register */
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
        else{
            /* do nothing */
        }
    }
    else{
        if(IRQNumber <= 31){
            /* Program ICER0 register */
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64){
            /* Program ICER1 register */
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96){
            /* Program ICER2 register */
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
        else{
            /* do nothing */
        }
    }
}

void Timer_IRQHandling(Timer_Handle_t* Timer_Handle){

    /* Check if TIM6 interrupt happened */
    if(Timer_Handle->pTimer->SR & (0x0001 << TIM6_7_SR_UIF)){
        /* Clear UIF bit in SR register */
        Timer_Handle->pTimer->SR &= ~(1 << TIM6_7_SR_UIF);
        /* Call application callback */
        Timer_ApplicationEventCallback();
    }
}

__attribute__((weak)) void Timer_ApplicationEventCallback(void){

    /* This is a weak implementation. The application may override this function */
}
