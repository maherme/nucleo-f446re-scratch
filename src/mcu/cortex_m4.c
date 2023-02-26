/********************************************************************************************************//**
* @file cortex_m4.c
*
* @brief File containing the APIs for configuring the Cortex M4.
*
* Public Functions:
*       - void IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       - void IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
*       - void EnableSleepOnExit(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#define SCR_REG     0xE000ED10

#include "cortex_m4.h"
#include "stm32f446xx.h"
#include <stdint.h>

void IRQConfig(uint8_t IRQNumber, uint8_t en_or_di){

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

void IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
    /* Find out the IPR register */
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift = (8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift);
}

void EnableSleepOnExit(void){

    SCB->SCR |= (1 << SCB_SCR_SLEEPONEXIT);
}