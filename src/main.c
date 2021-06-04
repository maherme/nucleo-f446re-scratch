#include <stdio.h>
#include <stdint.h>
#include "../inc/stm32f446xx.h"
#include "../inc/gpio_driver.h"

#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW

extern void initialise_monitor_handles(void);

static void delay(void){
    for(uint32_t i = 0; i < 500000; i++);
}

int main(void){

    initialise_monitor_handles();

    printf("Starting program!!!\n");

    GPIO_Handle_t GpioLed, GpioBtn;

    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULL;

    GPIO_PerClkCtrl(GPIOA, ENABLE);
    GPIO_Init(&GpioLed);

    GpioBtn.pGPIOx = GPIOC;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULL;

    GPIO_PerClkCtrl(GPIOC, ENABLE);
    GPIO_Init(&GpioBtn);

    for(;;){
        if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED){
            delay();
            GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
        }
    }

    return 0;
}
