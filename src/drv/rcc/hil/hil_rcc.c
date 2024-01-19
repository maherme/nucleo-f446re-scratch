/********************************************************************************************************//**
* @file hil_rcc.c
*
* @brief File containing the APIs for testing the RCC peripheral in the hardware.
*
* Public Functions:
*       - void SetHSEBypass(void)
*       - void SetPLLMax(void)
*       - void SetMCO_LSE_HSE(void)
*       - void SetMCO_PLL(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include <stdio.h>
#include <stdint.h>
#include "hil_rcc.h"
#include "rcc_driver.h"
#include "flash_driver.h"
#include "gpio_driver.h"

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

void SetMCO_LSE_HSE(void){

    RCC_Config_t RCC_Cfg = {0};
    GPIO_Handle_t MCOPin = {0};

    /* Set configuration */
    RCC_Cfg.mco1_source = MCO1_LSE;
    RCC_Cfg.mco1_presc = MCO_P_1;
    RCC_Cfg.lse_bypass = RCC_LSE_CRYSTAL;
    RCC_Cfg.mco2_source = MCO2_HSE;
    RCC_Cfg.mco2_presc = MCO_P_4;
    /* Set MCO1 */
    RCC_SetMCO1Clk(RCC_Cfg);
    /* Set MCO2 */
    RCC_SetMCO2Clk(RCC_Cfg);
    /* Set clock configuration */
    RCC_Cfg.clk_source = RCC_CLK_SOURCE_HSE;
    RCC_Cfg.hse_mode = RCC_HSE_BYPASS;
    /* Set clock */
    RCC_SetSystemClock(RCC_Cfg);

    /* Set GPIOs */
    MCOPin.pGPIOx = GPIOA;
    MCOPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    MCOPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    MCOPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULL;
    MCOPin.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
    MCOPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    /* MCO1 PA8 */
    MCOPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
    GPIO_Init(&MCOPin);
    /* MCO2 PC9 */
    MCOPin.pGPIOx = GPIOC;
    MCOPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    GPIO_Init(&MCOPin);
}

void SetMCO_PLL(void){

    RCC_Config_t RCC_Cfg = {0};
    GPIO_Handle_t MCOPin = {0};

    /* Set configuration */
    RCC_Cfg.clk_source = RCC_CLK_SOURCE_PLL_P;
    RCC_Cfg.ahb_presc = AHB_NO_PRESC;
    RCC_Cfg.apb1_presc = APB1_NO_PRESC;
    RCC_Cfg.apb2_presc = APB2_NO_PRESC;
    RCC_Cfg.mco1_source = MCO1_PLL;
    RCC_Cfg.mco1_presc = MCO_P_5;
    RCC_Cfg.pll_source = PLL_SOURCE_HSE;
    RCC_Cfg.pll_n = 80;
    RCC_Cfg.pll_m = 4;
    RCC_Cfg.pll_p = PLL_P_8;
    RCC_Cfg.mco2_source = MCO2_PLLI2S;
    RCC_Cfg.mco2_presc = MCO_P_5;
    RCC_Cfg.plli2s_n = 80;
    RCC_Cfg.plli2s_m = 4;
    /* Set MCO1 */
    RCC_SetMCO1Clk(RCC_Cfg);
    /* Set MCO2 */
    RCC_SetMCO2Clk(RCC_Cfg);
    /* Set clock */
    RCC_SetSystemClock(RCC_Cfg);

    /* Set GPIOs */
    MCOPin.pGPIOx = GPIOA;
    MCOPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    MCOPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    MCOPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULL;
    MCOPin.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
    MCOPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    /* MCO1 PA8 */
    MCOPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
    GPIO_Init(&MCOPin);
    /* MCO2 PC9 */
    MCOPin.pGPIOx = GPIOC;
    MCOPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    GPIO_Init(&MCOPin);
}
