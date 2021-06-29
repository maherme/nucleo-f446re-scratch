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

    /* SPI2 configuration */
    SPI2_GPIOInit();
    SPI2_Init();

    /* Enable the SPI2 SSOE for enabling the NSS output */
    /* The NSS pin is managed by the HW */
    SPI_SSOECfg(SPI2, ENABLE);

    for(;;);

    return 0;
}

void EXTI15_10_Handler(void){

    delay(); /* Prevent debouncing of button */

    GPIO_IRQHandling(GPIO_PIN_NO_13);

    /* Toggle LED */
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);


    //SPI2_SendHello();
    SPI2_ReadPinArd();
    delay();
    SPI2_SetPinArd();
    //SPI2_ReadANArd();
    delay();
    SPI2_ReadPinArd();
    delay();
    SPI2_PrintArd();
    delay();
    SPI2_ReadIDArd();
}
