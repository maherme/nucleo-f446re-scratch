/********************************************************************************************************//**
* @file test_pwr.c
*
* @brief File containing the APIs for testing the power peripheral.
*
* Public Functions:
*       - void Test_PWR_SetPLLMax(void)
*       - void Test_SleepOnExit(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include "test_pwr.h"
#include "pwr_driver.h"
#include "rcc_driver.h"
#include "flash_driver.h"
#include "timer_driver.h"
#include "stm32f446xx.h"
#include "cortex_m4.h"
#include <stdint.h>
#include <stdio.h>

#include "gpio_driver.h"

/** @brief Handler structure for Timer peripheral */
Timer_Handle_t Timer = {0};

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