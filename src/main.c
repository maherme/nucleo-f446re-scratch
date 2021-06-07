/*****************************************************************************************************
* FILENAME :        main.c
*
* DESCRIPTION :
*       File containing the main function.
*
**/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f446xx.h"
#include "gpio_driver.h"
#include "spi_driver.h"
#include "test.h"

#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW

extern void initialise_monitor_handles(void);

static void delay(void){
    for(uint32_t i = 0; i < 500000; i++);
}

int main(void){

    char user_data[] = "Hello world";

    initialise_monitor_handles();

    printf("Starting program!!!\n");

    /* LED configuration */
    LED_GPIOInit();

    /* Button configuration */
    Button_GPIOInit();

    /* IRQ configuration for button */
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIORITY15);
    GPIO_IRQConfig(IRQ_NO_EXTI15_10, ENABLE);

    /* SPI2 configuration */
    SPI2_GPIOInit();
    SPI2_Init();
    /* Put NSS signal internally high and avoids MODF error */
    SPI_SSICfg(SPI2, ENABLE);
    /* Enable the SPI2 peripheral */
    SPI_Enable(SPI2, ENABLE);

    /* Send data */
    SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

    /* Disable the SPI2 peripheral */
    SPI_Enable(SPI2, DISABLE);

    for(;;){
    }

    return 0;
}

void EXTI15_10_Handler(void){
    delay();
    GPIO_IRQHandling(GPIO_PIN_NO_13);
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}
