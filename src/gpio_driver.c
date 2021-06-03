#include "../inc/gpio_driver.h"

void GPIO_Init(GPIO_Handle_t* pGPIOHandle){

    uint32_t temp = 0;

    /* Configure the mode */
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
        /* Non-interrupt mode */
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER |= temp;
    }
    else{
        /* to do */
    }

    temp = 0;

    /* Configure the speed */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDER |= temp;
    temp = 0;

    /* Configure the pupd settings */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR |= temp;
    temp = 0;

    /* Configure the optype */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER |= temp;
    temp = 0;

    /* Configure the alternate functionality */
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
        /* to do */
    }
}

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx){
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
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx){
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pin_number, uint8_t value){
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t value){
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pin_number){
}

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t en_or_di){
}

void GPIO_IRQHandling(uint8_t pin_number){
}
