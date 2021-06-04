#include "../inc/gpio_driver.h"

void GPIO_Init(GPIO_Handle_t* pGPIOHandle){

    uint32_t temp = 0;

    /* Configure the mode */
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
        /* Non-interrupt mode */
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); /* clearing */
        pGPIOHandle->pGPIOx->MODER |= temp; /* setting */
    }
    else{
        /* to do */
    }

    temp = 0;

    /* Configure the speed */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); /* clearing */
    pGPIOHandle->pGPIOx->OSPEEDER |= temp; /* setting */
    temp = 0;

    /* Configure the pupd settings */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); /* clearing */
    pGPIOHandle->pGPIOx->PUPDR |= temp; /* setting */
    temp = 0;

    /* Configure the optype */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); /* clearing */
    pGPIOHandle->pGPIOx->OTYPER |= temp; /* setting */
    temp = 0;

    /* Configure the alternate functionality */
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

        uint8_t temp1, temp2 = 0;

        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
    }
}

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx){

    if(pGPIOx == GPIOA){
        GPIOA_REG_RESET();
    }
    else if(pGPIOx == GPIOB){
        GPIOB_REG_RESET();
    }
    else if(pGPIOx == GPIOC){
        GPIOC_REG_RESET();
    }
    else if(pGPIOx == GPIOD){
        GPIOD_REG_RESET();
    }
    else if(pGPIOx == GPIOE){
        GPIOE_REG_RESET();
    }
    else if(pGPIOx == GPIOF){
        GPIOF_REG_RESET();
    }
    else if(pGPIOx == GPIOG){
        GPIOG_REG_RESET();
    }
    else if(pGPIOx == GPIOH){
        GPIOH_REG_RESET();
    }
    else{
        /* do nothing */
    }
}

void GPIO_PerClkCtrl(GPIO_RegDef_t* pGPIOx, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        if(pGPIOx == GPIOA){
            GPIOA_PCLK_EN();
        }
        else if(pGPIOx == GPIOB){
            GPIOB_PCLK_EN();
        }
        else if(pGPIOx == GPIOC){
            GPIOC_PCLK_EN();
        }
        else if(pGPIOx == GPIOD){
            GPIOD_PCLK_EN();
        }
        else if(pGPIOx == GPIOE){
            GPIOE_PCLK_EN();
        }
        else if(pGPIOx == GPIOF){
            GPIOF_PCLK_EN();
        }
        else if(pGPIOx == GPIOG){
            GPIOG_PCLK_EN();
        }
        else if(pGPIOx == GPIOH){
            GPIOH_PCLK_EN();
        }
        else{
            /* do nothing */
        }
    }
    else{
        if(pGPIOx == GPIOA){
            GPIOA_PCLK_DI();
        }
        else if(pGPIOx == GPIOB){
            GPIOB_PCLK_DI();
        }
        else if(pGPIOx == GPIOC){
            GPIOC_PCLK_DI();
        }
        else if(pGPIOx == GPIOD){
            GPIOD_PCLK_DI();
        }
        else if(pGPIOx == GPIOE){
            GPIOE_PCLK_DI();
        }
        else if(pGPIOx == GPIOF){
            GPIOF_PCLK_DI();
        }
        else if(pGPIOx == GPIOG){
            GPIOG_PCLK_DI();
        }
        else if(pGPIOx == GPIOH){
            GPIOH_PCLK_DI();
        }
        else{
            /* do nothing */
        }
    }
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t pin_number){

    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> pin_number) & 0x00000001);

    return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx){

    uint16_t value;
    value = (uint16_t)pGPIOx->IDR;

    return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pin_number, uint8_t value){

    if(value == GPIO_PIN_SET){
        pGPIOx->ODR |= (1 << pin_number);
    }
    else{
        pGPIOx->ODR &= ~(1 << pin_number);
    }
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t value){

    pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pin_number){
    pGPIOx->ODR ^= (1 << pin_number);
}

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t en_or_di){
}

void GPIO_IRQHandling(uint8_t pin_number){
}
