/********************************************************************************************************//**
* @file test_rcc.c
*
* @brief File containing the APIs for testing the RCC peripheral.
*
* Public Functions:
*       - void SetHSEBypass(void)
*       - void SetPLLMax(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include <stdio.h>
#include <stdint.h>
#include "test_rcc.h"
#include "rcc_driver.h"
#include "flash_driver.h"

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void SetHSEBypass(void){

    uint32_t temp = 0;
    RCC_Config_t RCC_Cfg = {0};

    /* Set configuration */
    RCC_Cfg.clk_source = RCC_CLK_SOURCE_HSE;
    RCC_Cfg.hse_mode = RCC_HSE_BYPASS;
    /* Set clock */
    RCC_SetSystemClock(RCC_Cfg);

    /* Get clock values */
    temp = RCC_GetPCLK1Value();
    printf("PCLK1 Value: %ld\n", temp);
    temp = RCC_GetPCLK2Value();
    printf("PCLK2 Value: %ld\n", temp);
    temp = RCC_GetPLLOutputClock();
    printf("PLL Output Clock: %ld\n", temp);
}

void SetPLLMax(void){

    uint32_t temp = 0;
    RCC_Config_t RCC_Cfg = {0};

    /* Set FLASH latency according to clock frequency (see reference manual) */
    Flash_SetLatency(5);

    /* Set configuration */
    RCC_Cfg.clk_source = RCC_CLK_SOURCE_PLL_P;
    RCC_Cfg.pll_source = PLL_SOURCE_HSE;
    RCC_Cfg.ahb_presc = AHB_NO_PRESC;
    RCC_Cfg.apb1_presc = APB1_PRESC_4;
    RCC_Cfg.apb2_presc = APB2_PRESC_2;
    RCC_Cfg.pll_n = 180;
    RCC_Cfg.pll_m = 4;
    RCC_Cfg.pll_p = PLL_P_2;
    /* Set clock */
    RCC_SetSystemClock(RCC_Cfg);

    /* Get clock values */
    temp = RCC_GetPCLK1Value();
    printf("PCLK1 Value: %ld\n", temp);
    temp = RCC_GetPCLK2Value();
    printf("PCLK2 Value: %ld\n", temp);
    temp = RCC_GetPLLOutputClock();
    printf("PLL Output Clock: %ld\n", temp);
}
