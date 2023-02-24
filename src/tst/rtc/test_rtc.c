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
#include "cortex_m4.h"
#include <stdio.h>

#if TEST_RTC

/** @brief Set to 1 for testing using alarm interrupt */
#define TEST_RTC_IRQ    1

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
 * @brief Function for configuring and init the alarm in the RTC peripheral.
 * @return void.
 */
static void RTC_Test_Alarm_Init(void);

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

#if TEST_RTC_IRQ
    IRQConfig(IRQ_RTC_ALARM, ENABLE);
#endif

    RTC_Test_Alarm_Init();

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

static void RTC_Test_Alarm_Init(void){

    RTC_Alarm_t alarm_cfg = {0};

    alarm_cfg.AlarmSel = RTC_ALARM_A;
    alarm_cfg.DateMask = 1;
    alarm_cfg.WeekDaySelec = 0;
    alarm_cfg.DateTens = 0;
    alarm_cfg.DateUnits = 1;
    alarm_cfg.HoursMask = 1;
    alarm_cfg.PM = 0;
    alarm_cfg.HourTens = 1;
    alarm_cfg.HourUnits = 2;
    alarm_cfg.MinutesMask = 1;
    alarm_cfg.MinuteTens = 0;
    alarm_cfg.MinuteUnits = 0;
    alarm_cfg.SecondsMask = 0;
    alarm_cfg.SecondTens = 0;
    alarm_cfg.SecondUnits = 0;
#if !TEST_RTC_IRQ
    alarm_cfg.IRQ = 0;
#else
    alarm_cfg.IRQ = 1;
#endif

    RTC_SetAlarm(alarm_cfg);

    alarm_cfg.AlarmSel = RTC_ALARM_B;
    alarm_cfg.SecondTens = 3;
    alarm_cfg.SecondUnits = 0;

    RTC_SetAlarm(alarm_cfg);
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

#if !TEST_RTC_IRQ
    if(RTC_CheckAlarm(RTC_ALARM_A) == 1){
        printf("ALARM A!!!\n");
        RTC_ClearAlarm(RTC_ALARM_A);
    }

    if(RTC_CheckAlarm(RTC_ALARM_B) == 1){
        printf("ALARM B!!!\n");
        RTC_ClearAlarm(RTC_ALARM_B);
    }
#endif
}

static void RTC_Test_Timer6_Config(void){

    RTC_Test_Timer.tim_num = TIMER6;
    RTC_Test_Timer.pTimer = TIM6;
    RTC_Test_Timer.prescaler = 240;
    RTC_Test_Timer.period = 64000 - 1;

    Timer_Init(&RTC_Test_Timer);
    IRQConfig(IRQ_NO_TIM6_DAC, ENABLE);
    Timer_Start(&RTC_Test_Timer);
}

/***********************************************************************************************************/
/*                               Weak Function Overwrite Definitions                                       */
/***********************************************************************************************************/

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

#if TEST_RTC_IRQ

void RTC_Alarm_Handler(void){

    RTC_Alarm_IRQHandling();
}

void RTC_AlarmEventCallback(RTC_AlarmSel_t alarm){

    if(alarm == RTC_ALARM_A){
        printf("ALARM A!!!\n");
        RTC_ClearAlarm(RTC_ALARM_A);
        RTC_DisableAlarm(RTC_ALARM_A);
    }
    else if(alarm == RTC_ALARM_B){
        printf("ALARM B!!!\n");
        RTC_ClearAlarm(RTC_ALARM_B);
    }
    else{
        /* do nothing */
    }
}

#endif

#endif /* if TEST_RTC */