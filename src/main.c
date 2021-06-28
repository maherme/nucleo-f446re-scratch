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

    char user_data[] = "Hello world";
    uint8_t data_len = strlen(user_data);

    delay(); /* Prevent debouncing of button */

    GPIO_IRQHandling(GPIO_PIN_NO_13);

    /* Toggle LED */
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);

    /* Enable the SPI2 peripheral */
    SPI_Enable(SPI2, ENABLE);

    /* Send length information */
    SPI_SendData(SPI2, &data_len, 1);

    /* Send data */
    SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

    /* Confirm SPI is not busy */
    while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

    /* Disable the SPI2 peripheral */
    SPI_Enable(SPI2, DISABLE);
}
