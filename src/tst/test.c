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

#include "test.h"
#include "cortex_m4.h"
#include "stm32f446xx.h"
#include "gpio_driver.h"
#include "test_spi.h"
#include "test_i2c.h"
#include "test_usart.h"
#include "test_rcc.h"
#include "test_timer.h"
#include "test_dma.h"
#include "test_rtc.h"
#include "test_can.h"
#include "test_pwr.h"
#include "utils.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>

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
    IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIORITY15);
    IRQConfig(IRQ_NO_EXTI15_10, ENABLE);

#if TEST_SPI
    /* Configure and initialise SPI2 peripheral */
    SPI2_Config();
#endif

#if TEST_I2C
    /* Configure and initialise I2C1 peripheral */
    I2C1_Config();
#endif

#if TEST_USART
    /* Configure and initialise USART3 peripheral */
    USART3_Config();
#endif

#if TEST_RCC
    //SetHSEBypass();
    //SetPLLMax();
    //SetMCO_LSE_HSE();
    SetMCO_PLL();
#endif

#if TEST_TIMER
    //Timer6_Config();
    //Timer2_Config();
    Timer4_Config();
    //Timer3_Config();
#endif

#if TEST_DMA
    DMA1_Config();
#endif

#if TEST_RTC
    RTC_Test_Config();
#endif

#if TEST_CAN
    CAN1_Config();
#endif

#if TEST_PWR
    Test_SleepOnExit();
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

#if TEST_TIMER
    //Timer2_Process();
    //Timer3_Process();
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

#if TEST_DMA
    /* Send USART request to DMA controller if interrupts are disabled */
    //DMA1_USART3_Request();
    /* Send USART request to DMA controller if interrupts are enabled */
    DMA1_USART3_Request_IT();
#endif

#if TEST_RTC
    RTC_Test_Reset();
#endif

#if TEST_CAN
    CAN1_Send();
    //CAN1_Send_Receive();
#endif

#if TEST_PWR
    //Test_PWR_SetPLLMax();
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
