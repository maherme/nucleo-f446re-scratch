/********************************************************************************************************//**
* @file test_rtc.c
*
* @brief File containing the APIs for testing the RTC peripheral.
*
* Public Functions:
*       - void RTC_Test_Config(void)
*       - void RTC_Test_Reset(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include "test_rtc.h"
#include "test.h"
#include "rtc_driver.h"
#include "timer_driver.h"
#include "stm32f446xx.h"
#include <stdio.h>

/** @brief Structure for RTC configuration */
RTC_Config_t RTC_Cfg = {0};
/** @brief Handler structure for Timer peripheral */
Timer_Handle_t RTC_Test_Timer = {0};

/***********************************************************************************************************/
/*                                       Static Function Prototypes                                        */
/***********************************************************************************************************/

/**
 * @brief Function for initializing the RTC peripheral.
 * @return void.
 */
static void RTC_Test_Init(void);

/**
 * @brief Function for configuring the timer TIM6
 * @return void.
 */
static void RTC_Test_Timer6_Config(void);

/**
 * @brief Function for requesting date and time to the RTC peripheral. The inforation is printed using
 *        printf function.
 * @return void.
 */
static void RTC_Test_Request(void);

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void RTC_Test_Config(void){

    /* Turn on required input clock */
    RCC->CSR |= (1 << 0);
    while(!(RCC->CSR & (1 << 1)));

    /* Select clock source */
    RTC_ClkSource(RCC_LSI_SOURCE);

    RTC_PerClkCtrl(ENABLE);

    RTC_Test_Init();

    RTC_Init(RTC_Cfg);

    RTC_Test_Timer6_Config();
}

void RTC_Test_Reset(void){

    RTC_Init(RTC_Cfg);
}

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/

static void RTC_Test_Init(void){

    RTC_Cfg.RTC_HoursFormat = RTC_AM_PM;
    RTC_Cfg.RTC_Time.SecondUnits = 0;
    RTC_Cfg.RTC_Time.SecondTens = 5;
    RTC_Cfg.RTC_Time.MinuteUnits = 9;
    RTC_Cfg.RTC_Time.MinuteTens = 5;
    RTC_Cfg.RTC_Time.HourUnits = 1;
    RTC_Cfg.RTC_Time.HourTens = 1;
    RTC_Cfg.RTC_Time.PM = 1;
    RTC_Cfg.RTC_Date.YearUnits = 8;
    RTC_Cfg.RTC_Date.YearTens = 9;
    RTC_Cfg.RTC_Date.MonthUnits = 2;
    RTC_Cfg.RTC_Date.MonthTens = 1;
    RTC_Cfg.RTC_Date.DateUnits = 1;
    RTC_Cfg.RTC_Date.DateTens = 3;
}

static void RTC_Test_Request(void){

    RTC_Time_t time = {0};
    RTC_Date_t date = {0};
    const char pm[3] = {'P', 'M', '\0'};
    const char am[3] = {'A', 'M', '\0'};
    const char* pm_am = NULL;

    RTC_GetTime(&time);
    RTC_GetDate(&date);

    if(time.PM){
        pm_am = pm;
    }
    else{
        pm_am = am;
    }

    printf("Time: %d%d:%d%d:%d%d %s\n",
            time.HourTens, time.HourUnits,
            time.MinuteTens, time.MinuteUnits,
            time.SecondTens, time.SecondUnits,
            pm_am);
    printf("Date: %d%d-%d%d-%d%d\n",
            date.YearTens, date.YearUnits,
            date.MonthTens, date.MonthUnits,
            date.DateTens, date.DateUnits);
}

static void RTC_Test_Timer6_Config(void){

    RTC_Test_Timer.tim_num = TIMER6;
    RTC_Test_Timer.pTimer = TIM6;
    RTC_Test_Timer.prescaler = 240;
    RTC_Test_Timer.period = 64000 - 1;

    Timer_Init(&RTC_Test_Timer);
    Timer_IRQConfig(IRQ_NO_TIM6_DAC, ENABLE);
    Timer_Start(&RTC_Test_Timer);
}

/***********************************************************************************************************/
/*                               Weak Function Overwrite Definitions                                       */
/***********************************************************************************************************/

#if !TEST_TIMER

void TIM6_DAC_Handler(void){
    Timer_IRQHandling(&RTC_Test_Timer);
}

void Timer_ApplicationEventCallback(Timer_Num_t tim_num, Timer_Event_t timer_event){

    if(timer_event == TIMER_UIF_EVENT){
        if(tim_num == TIMER6){
            RTC_Test_Request();
        }
    }
    else{
        /* do nothing */
    }
}

#endif
