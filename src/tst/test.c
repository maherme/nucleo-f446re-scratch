/********************************************************************************************************//**
* @file test.c
*
* @brief File containing configuring function for testing drivers.
*
* PUBLIC FUNCTIONS :
*       - void    test_init(void)
*       - void    test_process(void)
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
#include "test_spi.h"
#include "test_i2c.h"
#include "test_usart.h"
#include "test_rcc.h"
#include "test_timer.h"
#include "utils.h"

/**
 * @name Flags for enabling peripheral testing.
 * @{
 */
#define TEST_SPI    0   /**< @brief Set to 1 for enabling the SPI test */
#define TEST_I2C    0   /**< @brief Set to 1 for enabling the I2C test */
#define TEST_USART  0   /**< @brief Set to 1 for enabling the USART test */
#define TEST_RCC    1   /**< @brief Set to 1 for enabling the RCC test */
#define TEST_TIMER  0   /**< @brief Set to 1 for enabling the Timer test */
/** @} */

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

#if TEST_SPI
    /* Configure and initialise SPI2 peripheral */
    SPI2_Config();
#endif

#if TEST_I2C
    /* Configure and initialise I2C1 peripheral */
    I2C1_Config();
#endif

#if TEST_USART
    /* Configure and initialise USART2 peripheral */
    USART3_Config();
#endif

#if TEST_RCC
    //SetHSEBypass();
    //SetPLLMax();
    SetMCO();
#endif

#if TEST_TIMER
    Timer6_Config();
#endif
}

void test_process(void){
#if TEST_RCC
    static uint32_t i = 0;
    if(i > 200000){
        /* Toggle LED */
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
        i = 0;
    }
    i++;
#endif
}

/***********************************************************************************************************/
/*                               Weak Function Overwrite Definitions                                       */
/***********************************************************************************************************/

void EXTI9_5_Handler(void){

#if TEST_SPI
    /* Interrupt actions for testing SPI */
    SPI_IRQActions();
#endif
}

void EXTI15_10_Handler(void){

    delay(); /* Prevent debouncing of button */

    GPIO_IRQHandling(GPIO_PIN_NO_13);

#if TEST_SPI
    /* Send a "hello world" string via SPI to Arduino board */
    //SPI2_SendHello();
    /* Send commands via SPI to Arduino board */
    SPI_SendCmds();
#endif

#if TEST_I2C
    /* Send "hello world" string via I2C to Arduino board */
    //I2C1_SendHello();
    /* Send commands via I2C to Arduino board using non interrupt mode */
    //I2C1_SendCmd();
    /* Send commands via I2C to Arduino board using interrupt mode */
    I2C1_SendCmdIT();
#endif

#if TEST_USART
    /* Send USART data */
    //USART3_SendHello();
    USART3_TxRx();
#endif

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
