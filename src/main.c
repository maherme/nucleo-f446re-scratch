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

extern void initialise_monitor_handles(void);

int main(void){

    initialise_monitor_handles();

    printf("Starting program!!!\n");

    /* LED configuration */
    LED_GPIOInit();

    /* Button configuration */
    Button_GPIOInit();

    /* IRQ configuration for button */
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIORITY15);
    GPIO_IRQConfig(IRQ_NO_EXTI15_10, ENABLE);

    /* Configure and initialise SPI2 peripheral */
    //SPI2_Config();

    /* Configure and initialise I2C1 peripheral */
    I2C1_Config();

    for(;;){
    }

    return 0;
}

void EXTI9_5_Handler(void){

    /* Interrupt actions for SPI */
    //SPI_IRQActions();
}

void EXTI15_10_Handler(void){

    delay(); /* Prevent debouncing of button */

    GPIO_IRQHandling(GPIO_PIN_NO_13);

    /* Send commands via SPI if button is pressed */
    //SPI_SendCmds();

    /* Send I2C data */
    //I2C1_SendHello();
    //I2C1_SendCmd();
    //I2C1_SendCmdIT();

    /* Toggle LED */
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}
