/*****************************************************************************************************
* FILENAME :        i2c_driver.c
*
* DESCRIPTION :
*       File containing the APIs for configuring the I2C peripheral.
*
* PUBLIC FUNCTIONS :
*       void    I2C_Init(I2C_Handle_t* pI2C_Handle)
*       void    I2C_DeInit(I2C_RegDef_t* pI2Cx)
*       void    I2C_PerClkCtrl(I2C_RegDef_t* pI2Cx, uint8_t en_or_di)
*       void    I2C_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       void    I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
*       void    I2C_Enable(I2C_RegDef_t* pI2Cx, uint8_t en_or_di)
*       uint8_t I2C_GetFlagStatus(SPI_RegDef_t* pI2Cx, uint32_t flagname)
*       void    I2C_ApplicationEventCallback(I2C_Handle_t* pI2C_Handle, uint8_t app_event)
*
* NOTES :
*       For further information about functions refer to the corresponding header file.
*
**/

#include <stdint.h>
#include "i2c_driver.h"

void I2C_Init(I2C_Handle_t* pI2C_Handle){
}

void I2C_DeInit(I2C_RegDef_t* pI2Cx){

    if(pI2Cx == I2C1){
        I2C1_REG_RESET();
    }
    else if(pI2Cx == I2C2){
        I2C2_REG_RESET();
    }
    else if(pI2Cx == I2C3){
        I2C3_REG_RESET();
    }
    else{
        /* do nothing */
    }
}

void I2C_PerClkCtrl(I2C_RegDef_t* pI2Cx, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        if(pI2Cx == I2C1){
            I2C1_PCLK_EN();
        }
        else if(pI2Cx == I2C2){
            I2C2_PCLK_EN();
        }
        else if(pI2Cx == I2C3){
            I2C3_PCLK_EN();
        }
        else{
            /* do nothing */
        }
    }
    else{
        if(pI2Cx == I2C1){
            I2C1_PCLK_DI();
        }
        else if(pI2Cx == I2C2){
            I2C2_PCLK_DI();
        }
        else if(pI2Cx == I2C3){
            I2C3_PCLK_DI();
        }
        else{
            /* do nothing */
        }
    }
}

void I2C_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di){

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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
    /* Find out the IPR register */
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift = (8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift);
}

void I2C_Enable(I2C_RegDef_t* pI2Cx, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    }
    else{
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }
}

uint8_t I2C_GetFlagStatus(SPI_RegDef_t* pI2Cx, uint32_t flagname){
    return 0;
}

 __attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t* pI2C_Handle, uint8_t app_event){

    /* This is a weak implementation. The application may override this function */
}
