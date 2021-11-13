/********************************************************************************************************//**
* @file test.c
*
* @brief File containing configuring function for testing drivers.
*
* PUBLIC FUNCTIONS :
*       - void    test_init(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32f446xx.h"
#include "gpio_driver.h"
#include "test.h"
#include "utils.h"

/***********************************************************************************************************/
/*                                       Static Function Prototypes                                        */
/***********************************************************************************************************/

/**
 * @brief Function to initialize GPIO port for the LED.
 * @return void.
 */
static void LED_GPIOInit(void);

/**
 * @brief Function to initialize GPIO port for the button.
 * @return void.
 */
static void Button_GPIOInit(void);

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void test_init(void){

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
    //I2C1_Config();

    /* Configure and initialise USART2 peripheral */
    //USART3_Config();
}

/***********************************************************************************************************/
/*                               Weak Function Overwrite Definitions                                       */
/***********************************************************************************************************/

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

    /* Send USART data */
    //USART3_SendHello();
    //USART3_TxRx();

    /* Toggle LED */
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/

static void LED_GPIOInit(void){

    GPIO_Handle_t GpioLed;

    memset(&GpioLed, 0, sizeof(GpioLed));

    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULL;

    GPIO_Init(&GpioLed);
}

static void Button_GPIOInit(void){

    GPIO_Handle_t GpioBtn;

    memset(&GpioBtn, 0, sizeof(GpioBtn));

    GpioBtn.pGPIOx = GPIOC;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;

    GPIO_Init(&GpioBtn);
}
