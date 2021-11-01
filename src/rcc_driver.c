/********************************************************************************************************//**
* @file rcc_driver.c
*
* @brief File containing the APIs for configuring the RCC peripheral.
*
* Public Functions:
*       - uint32_t RCC_GetPCLK1Value(void)
*       - uint32_t RCC_GetPCLK2Value(void)
*       - uint32_t RCC_GetPLLOutputClock(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
*/

#include <stdint.h>
#include "stm32f446xx.h"
#include "rcc_driver.h"

/** @brief Frequency of 8 MHz */
#define FREQ_8MHZ    8000000
/** @brief Frequency of 16 MHz */
#define FREQ_16MHZ   16000000

/** @brief Possible AHB prescaler values */
static uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
/** @brief Possible APB prescaler values */
static uint8_t APB_PreScaler[4] = {2, 4, 8, 16};

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

uint32_t RCC_GetPCLK1Value(void){

    uint32_t pclk1, systemclk, clksrc;
    uint8_t temp;
    uint8_t ahbp, apb1p;

    /* Check for SWS */
    clksrc = ((RCC->CFGR >> 2) & 0x3);

    if(clksrc == 0){
        /* clk is HSI */
        systemclk = FREQ_16MHZ;
    }
    else if(clksrc == 1){
        /* clk is HSE */
        systemclk = FREQ_8MHZ;
    }
    else if(clksrc == 2){
        /* clk is PLL */
        systemclk = RCC_GetPLLOutputClock();
    }
    else{
        /* do nothing */
    }

    /* AHB prescaler */
    temp = ((RCC->CFGR >> 4) & 0xF);

    if(temp < 8){
        ahbp = 1;
    }
    else{
        ahbp = AHB_PreScaler[temp-8];
    }

    /* APB1 prescaler */
    temp = ((RCC->CFGR >> 10) & 0x7);

    if(temp < 4){
        apb1p = 1;
    }
    else{
        apb1p = APB_PreScaler[temp-4];
    }

    pclk1 = (systemclk/ahbp)/apb1p;

    return pclk1;
}

uint32_t RCC_GetPCLK2Value(void){

    uint32_t pclk2, systemclk, clksrc;
    uint8_t temp;
    uint8_t ahbp, apb2p;

    /* Check for SWS */
    clksrc = ((RCC->CFGR >> 2) & 0x3);

    if(clksrc == 0){
        /* clk is HSI */
        systemclk = FREQ_16MHZ;
    }
    else if(clksrc == 1){
        /* clk is HSE */
        systemclk = FREQ_8MHZ;
    }
    else if(clksrc == 2){
        /* clk is PLL */
        systemclk = RCC_GetPLLOutputClock();
    }
    else{
        /* do nothing */
    }

    /* AHB prescaler */
    temp = ((RCC->CFGR >> 4) & 0xF);

    if(temp < 8){
        ahbp = 1;
    }
    else{
        ahbp = AHB_PreScaler[temp-8];
    }

    /* APB2 prescaler */
    temp = ((RCC->CFGR >> 13) & 0x7);

    if(temp < 4){
        apb2p = 1;
    }
    else{
        apb2p = APB_PreScaler[temp-4];
    }

    pclk2 = (systemclk/ahbp)/apb2p;

    return pclk2;
}

uint32_t RCC_GetPLLOutputClock(void){
    /* TO BE DONE */
    return 0;
}
