/********************************************************************************************************//**
* @file rtc_driver.c
*
* @brief File containing the APIs for configuring the RTC peripheral.
*
* Public Functions:
*       - void RTC_PerClkCtrl(uint8_t en_or_di)
*       - void RTC_ClkSource(RTC_ClkSource_t clk_source)
*       - void RTC_Init(RTC_Config_t RTC_Cfg)
*       - void RTC_SetTime(RTC_Time_t time)
*       - void RTC_GetTime(RTC_Time_t* time)
*       - void RTC_SetDate(RTC_Date_t date)
*       - void RTC_GetDate(RTC_Date_t* date)
*       - void RTC_ClearRSF(void)
*       - uint8_t RTC_GetRSF(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
*/

#include "rtc_driver.h"
#include "stm32f446xx.h"
#include <stdint.h>
#include "utils.h"

/***********************************************************************************************************/
/*                                       Static Function Prototypes                                        */
/***********************************************************************************************************/

/**
 * @brief Function for unlocking the RTC write protected registers.
 * @return void.
 */
static void RTC_Unlock(void);

/**
 * @brief Function for entering in configuration mode for the RTC.
 * @return void.
 */
static void RTC_EnterConfig(void);

/**
 * @brief Function for exiting from configuration mode for the RTC.
 * @return void.
 */
static void RTC_ExitConfig(void);

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void RTC_PerClkCtrl(uint8_t en_or_di){

    if(en_or_di == ENABLE){
        RTC_PCLK_EN();
    }
    else{
        RTC_PCLK_DI();
    }
}

void RTC_ClkSource(RTC_ClkSource_t clk_source){

    /* Disable backup protection */
    PWR_PCLK_EN();
    PWR->CR |= (1 << PWR_CR_DBP);

    RCC->BDCR &= ~(0x3 << RCC_BDCR_RTCSEL);
    RCC->BDCR |= (clk_source << RCC_BDCR_RTCSEL);
}

void RTC_Init(RTC_Config_t RTC_Cfg){

    uint32_t temp = 0;

    RTC_Unlock();
    RTC_EnterConfig();

    /* Set hours format */
    RTC->CR &= ~(1 << RTC_CR_FMT);
    RTC->CR |= (RTC_Cfg.RTC_HoursFormat << RTC_CR_FMT);

    temp |= ((RTC_Cfg.RTC_Time.SecondUnits << RTC_TR_SU) |
            (RTC_Cfg.RTC_Time.SecondTens << RTC_TR_ST) |
            (RTC_Cfg.RTC_Time.MinuteUnits << RTC_TR_MNU) |
            (RTC_Cfg.RTC_Time.MinuteTens << RTC_TR_MNT) |
            (RTC_Cfg.RTC_Time.HourUnits << RTC_TR_HU) |
            (RTC_Cfg.RTC_Time.HourTens << RTC_TR_HT) |
            (RTC_Cfg.RTC_Time.PM << RTC_TR_PM));

    /* Set time */
    RTC->TR &= ~(0xFFFFFFFF);
    RTC->TR = temp;

    temp = 0;
    temp |= ((RTC_Cfg.RTC_Date.DateUnits << RTC_DR_DU) |
            (RTC_Cfg.RTC_Date.DateTens << RTC_DR_DT) |
            (RTC_Cfg.RTC_Date.MonthUnits << RTC_DR_MU) |
            (RTC_Cfg.RTC_Date.MonthTens << RTC_DR_MT) |
            (RTC_Cfg.RTC_Date.WeekDayUnits << RTC_DR_WDU) |
            (RTC_Cfg.RTC_Date.YearUnits << RTC_DR_YU) |
            (RTC_Cfg.RTC_Date.YearTens << RTC_DR_YT));

    /* Set date */
    RTC->DR &= ~(0xFFFFFFFF);
    RTC->DR = temp;

    RTC_ExitConfig();
}

void RTC_SetTime(RTC_Time_t time){

    uint32_t temp = 0;

    temp |= ((time.SecondUnits << RTC_TR_SU) |
            (time.SecondTens << RTC_TR_ST) |
            (time.MinuteUnits << RTC_TR_MNU) |
            (time.MinuteTens << RTC_TR_MNT) |
            (time.HourUnits << RTC_TR_HU) |
            (time.HourTens << RTC_TR_HT) |
            (time.PM << RTC_TR_PM));

    RTC_Unlock();
    RTC_EnterConfig();

    /* Set time */
    RTC->TR &= ~(0xFFFFFFFF);
    RTC->TR = temp;

    RTC_ExitConfig();
}

void RTC_GetTime(RTC_Time_t* time){

    time->SecondUnits = (RTC->TR >> RTC_TR_SU) & 0xF;
    time->SecondTens = (RTC->TR >> RTC_TR_ST) & 0x7;
    time->MinuteUnits = (RTC->TR >> RTC_TR_MNU) & 0xF;
    time->MinuteTens = (RTC->TR >> RTC_TR_MNT) & 0x7;
    time->HourUnits = (RTC->TR >> RTC_TR_HU) & 0xF;
    time->HourTens = (RTC->TR >> RTC_TR_HT) & 0x3;
    time->PM = (RTC->TR >> RTC_TR_PM) & 0x1;
}

void RTC_SetDate(RTC_Date_t date){

    uint32_t temp = 0;

    temp |= ((date.DateUnits << RTC_DR_DU) |
            (date.DateTens << RTC_DR_DT) |
            (date.MonthUnits << RTC_DR_MU) |
            (date.MonthTens << RTC_DR_MT) |
            (date.WeekDayUnits << RTC_DR_WDU) |
            (date.YearUnits << RTC_DR_YU) |
            (date.YearTens << RTC_DR_YT));

    RTC_Unlock();
    RTC_EnterConfig();

    /* Set date */
    RTC->DR &= ~(0xFFFFFFFF);
    RTC->DR = temp;

    RTC_ExitConfig();
}

void RTC_GetDate(RTC_Date_t* date){

    date->DateUnits = (RTC->DR >> RTC_DR_DU) & 0xF;
    date->DateTens = (RTC->DR >> RTC_DR_DT) & 0x3;
    date->WeekDayUnits = (RTC->DR >> RTC_DR_WDU) & 0x7;
    date->MonthUnits = (RTC->DR >> RTC_DR_MU) & 0xF;
    date->MonthTens = (RTC->DR >> RTC_DR_MT) & 0x1;
    date->YearUnits = (RTC->DR >> RTC_DR_YU) & 0xF;
    date->YearTens = (RTC->DR >> RTC_DR_YT) & 0xF;
}

void RTC_ClearRSF(void){

    RTC->ISR &= ~(1 << RTC_ISR_RSF);
}

uint8_t RTC_GetRSF(void){

    uint8_t ret = 0;

    if(RTC->ISR & (1 << RTC_ISR_RSF)){
       ret = 1;
    }

    return ret;
}

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/

static void RTC_Unlock(void){

    RTC->WPR |= 0xCA;
    RTC->WPR |= 0x53;
}

static void RTC_EnterConfig(void){

    /* Set INIT bit to 1 in the RTC_ISR register to enter initialization mode */
    RTC->ISR |= (1 << RTC_ISR_INIT);
    /* Poll INITF bit of in the RTC_ISR register */
    while(!(RTC->ISR & (1 << RTC_ISR_INITF)));
}

static void RTC_ExitConfig(void){

    /* Exit the initialization mode by clearing the INIT bit */
    RTC->ISR &= ~(1 << RTC_ISR_INIT);
}
