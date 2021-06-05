#include <stdio.h>
#include <stdint.h>
#include <string.h>
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

    memset(&GpioLed, 0, sizeof(GpioLed));
    memset(&GpioBtn, 0, sizeof(GpioBtn));

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
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;

    GPIO_PerClkCtrl(GPIOC, ENABLE);
    GPIO_Init(&GpioBtn);

    /* IRQ configuration */
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIORITY15);
    GPIO_IRQConfig(IRQ_NO_EXTI15_10, ENABLE);

    for(;;){
    }

    return 0;
}

void EXTI15_10_Handler(void){
    delay();
    GPIO_IRQHandling(GPIO_PIN_NO_13);
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}
