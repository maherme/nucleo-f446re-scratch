/********************************************************************************************************//**
* @file test_pwr.c
*
* @brief File containing the APIs for testing the power peripheral.
*
* Public Functions:
*       - void Test_PWR_SetPLLMax(void)
*       - void Test_SleepOnExit(void)
*       - void Test_WFE_init(void)
*       - void Test_WFE_process(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include "test_pwr.h"
#include "pwr_driver.h"
#include "rcc_driver.h"
#include "flash_driver.h"
#include "timer_driver.h"
#include "gpio_driver.h"
#include "usart_driver.h"
#include "stm32f446xx.h"
#include "cortex_m4.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "gpio_driver.h"

/** @brief Handler structure for Timer peripheral */
Timer_Handle_t Timer = {0};
/** @brief Handler structure for USART peripheral */
USART_Handle_t USART3Handle;

/***********************************************************************************************************/
/*                                       Static Function Prototypes                                        */
/***********************************************************************************************************/

/**
 * @brief Function to initialize USART3 peripheral.
 * @return void.
 */
static void USART3_Init(USART_Handle_t* pUSART_Handle);

/**
 * @brief Function to initialize GPIO port for the USART3 peripheral.
 * @return void.
 *
 * @note
 *      PC10 -> USART3 TX
 *      PC11 -> USART3 RX
 *      Alt function mode -> 7
 */
static void USART3_GPIOInit(void);

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void Test_PWR_SetPLLMax(void){

    uint32_t temp = 0;
    RCC_Config_t RCC_Cfg = {0};

    /* Set configuration */
    RCC_Cfg.clk_source = RCC_CLK_SOURCE_PLL_P;
    RCC_Cfg.pll_source = PLL_SOURCE_HSE;
    RCC_Cfg.ahb_presc = AHB_NO_PRESC;
    RCC_Cfg.apb1_presc = APB1_PRESC_4;
    RCC_Cfg.apb2_presc = APB2_PRESC_2;
    RCC_Cfg.pll_n = 180;
    RCC_Cfg.pll_m = 4;
    RCC_Cfg.pll_p = PLL_P_2;

    /* Set VOS to mode 1 */
    PWR_SetRegVoltageScal(VOS_SCALE_1);
    /* Enable pwr clock */
    PWR_PCLK_EN();
    /* Set over-drive mode */
    PWR_SetOverDrive();
    /* Set FLASH latency according to clock frequency (see reference manual) */
    Flash_SetLatency(5);
    /* Set PLL to system clock */
    RCC_SetSystemClock(RCC_Cfg);

    /* Print PLL frequency */
    temp = RCC_GetPLLOutputClock();
    printf("PLL Output Clock: %ld\n", temp);
}

void Test_SleepOnExit(void){

    /* Configure TIMER 6 */
    Timer.tim_num = TIMER6;
    Timer.pTimer = TIM6;
    Timer.prescaler = 4999;
    Timer.period = 6400 - 1;

    Timer_Init(&Timer);
    IRQConfig(IRQ_NO_TIM6_DAC, ENABLE);
    /* Enable sleep on exit cortex mode */
    EnableSleepOnExit();
    Timer_Start(&Timer);
}

void Test_WFE_init(void){

    /* Configure USART3 */
    USART3_GPIOInit();
    USART3_Init(&USART3Handle);
    USART_Enable(USART3, ENABLE);
    /* Disable IRQ for button */
    IRQConfig(IRQ_NO_EXTI15_10, DISABLE);
    /* Set SEVONPEND bit in cortex */
    EnableSEVONPEND();
}

void Test_WFE_process(void){

    const char msg_awake[] = "Hello I am awake\n";
    const char msg_sleep[] = "Bye I go to sleep\n";

    /* Notify sleep */
    USART_SendData(&USART3Handle, (uint8_t*)msg_sleep, strlen(msg_sleep));
    /* Enter in low power mode */
    Enter_WFE();
    /* Notify awake */
    USART_SendData(&USART3Handle, (uint8_t*)msg_awake, strlen(msg_awake));
    /* Clear event pending bit */
    GPIO_IRQHandling(GPIO_PIN_NO_13);
    /* Clear interrupt pending in NVIC */
    IRQClearPending(IRQ_NO_EXTI15_10);
}

/***********************************************************************************************************/
/*                               Weak Function Overwrite Definitions                                       */
/***********************************************************************************************************/

void TIM6_DAC_Handler(void){
    Timer_IRQHandling(&Timer);
}

void Timer_ApplicationEventCallback(Timer_Num_t tim_num, Timer_Event_t timer_event){

    if(timer_event == TIMER_UIF_EVENT){
        if(tim_num == TIMER6){
            printf("Hello I am awake\n");
        }
    }
    else{
        /* do nothing */
    }
}

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/

static void USART3_Init(USART_Handle_t* pUSART_Handle){

    memset(pUSART_Handle, 0, sizeof(*pUSART_Handle));

    pUSART_Handle->pUSARTx = USART3;
    pUSART_Handle->USART_Config.USART_Baud = USART_STD_BAUD_115200;
    pUSART_Handle->USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    pUSART_Handle->USART_Config.USART_Mode = USART_MODE_TXRX;
    pUSART_Handle->USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    pUSART_Handle->USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    pUSART_Handle->USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

    USART_Init(pUSART_Handle);
}

static void USART3_GPIOInit(void){

    GPIO_Handle_t USARTPins;

    memset(&USARTPins, 0, sizeof(USARTPins));

    USARTPins.pGPIOx = GPIOC;
    USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    USARTPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    USARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

    /* USART3 TX */
    USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
    GPIO_Init(&USARTPins);

    /* USART3 RX */
    USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
    GPIO_Init(&USARTPins);
}