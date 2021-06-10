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

#define FREQ_8MHZ    8000000
#define FREQ_16MHZ   16000000

static uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
static uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};

/**
 * @fn RCC_GetPCLK1Value
 *
 * @brief function to calculate the PCLK1 clock value.
 *
 * @return PCLK1 clock value.
 */
static uint32_t RCC_GetPCLK1Value(void);

/**
 * @fn RCC_GetPLLOutputClock
 *
 * @brief function to calculate the PLL clock value.
 *
 * @return PLL clock value.
 */

static uint32_t RCC_GetPLLOutputClock(void);

void I2C_Init(I2C_Handle_t* pI2C_Handle){

    uint8_t temp = 0;
    uint16_t ccr_value = 0;

    /* ACK control bit */
    temp |= (pI2C_Handle->I2C_Config.I2C_ACKControl << 10);
    pI2C_Handle->pI2Cx->CR1 = temp;

    /* FREQ field of CR2 */
    temp = 0;
    temp |= RCC_GetPCLK1Value()/1000000U;
    pI2C_Handle->pI2Cx->CR2 = temp & 0x3F;

    /* Device own address */
    temp = 0;
    temp |= (pI2C_Handle->I2C_Config.I2C_DeviceAddress << 1);
    temp |= (1 << 14);
    pI2C_Handle->pI2Cx->OAR1 = temp;

    /* CCR */
    temp = 0;
    if(pI2C_Handle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
        /* standard mode */
        ccr_value = RCC_GetPCLK1Value()/(2*pI2C_Handle->I2C_Config.I2C_SCLSpeed);
        temp |= (ccr_value & 0xFFF);
    }
    else{
        /* fast mode */
        temp |= (1 << 15);
        temp |= (pI2C_Handle->I2C_Config.I2C_FMDutyCycle << 14);
        if(pI2C_Handle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
            ccr_value = RCC_GetPCLK1Value()/(3*pI2C_Handle->I2C_Config.I2C_SCLSpeed);
        }
        else{
            ccr_value = RCC_GetPCLK1Value()/(25*pI2C_Handle->I2C_Config.I2C_SCLSpeed);
        }
        temp |= (ccr_value & 0xFFF);
    }
    pI2C_Handle->pI2Cx->CCR = temp;
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

static uint32_t RCC_GetPCLK1Value(void){

    uint32_t pclk1, systemclk, clksrc;
    uint8_t temp;
    uint8_t ahbp, apb1p;

    /* Check for SWS */
    clksrc = ((RCC->CFGR >> 2) & 0x3);

    if(clksrc == 0){
        /* clk is HSI */
        systemclk = FREQ_16MHZ;
    }
    else if(clksrc == 1){
        /* clk is HSE */
        systemclk = FREQ_8MHZ;
    }
    else if(clksrc == 2){
        /* clk is PLL */
        systemclk = RCC_GetPLLOutputClock();
    }
    else{
        /* do nothing */
    }

    /* AHB prescaler */
    temp = ((RCC->CFGR >> 4) & 0xF);

    if(temp < 8){
        ahbp = 1;
    }
    else{
        ahbp = AHB_PreScaler[temp-8];
    }

    /* APB1 prescaler */
    temp = ((RCC->CFGR >> 10) & 0x7);

    if(temp < 4){
        apb1p = 1;
    }
    else{
        apb1p = APB1_PreScaler[temp-4];
    }

    pclk1 = (systemclk/ahbp)/apb1p;

    return pclk1;
}

static uint32_t RCC_GetPLLOutputClock(void){
    /* TO BE DONE */
    return 0;
}
