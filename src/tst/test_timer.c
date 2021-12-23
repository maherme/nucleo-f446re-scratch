/********************************************************************************************************//**
* @file test_timer.c
*
* @brief File containing the APIs for testing the Timer peripheral.
*
* Public Functions:
*       - void    Timer6_Config(void)
*       - void    Timer2_Config(void)
*       - void    Timer2_Process(void)
*       - void    Timer4_Config(void)
*       - void    Timer5_Config(void)
*       - void    Timer5_Process(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include <stdio.h>
#include "test_timer.h"
#include "timer_driver.h"
#include "gpio_driver.h"
#include "rcc_driver.h"
#include "stm32f446xx.h"
#include "utils.h"

/** @brief Handler structure for Timer peripheral */
Timer_Handle_t Timer = {0};
/** @brief Variable for managing input capture event */
static uint8_t capture_done = 0;
/** @brief Array for storing captured values by input capture */
static uint32_t input_capture[2] = {0};
/** @brief Variable for setting output compare frequency to 80Hz with a timer clock of 8MHz*/
static uint32_t pulse_value1 = 50000;
/** @brief Variable for setting output compare frequency to 160Hz with a timer clock of 8MHz */
static uint32_t pulse_value2 = 25000;
/** @brief Variable for setting output compare frequency to 320Hz with a timer clock of 8MHz */
static uint32_t pulse_value3 = 12500;
/** @brief Variable for setting output compare frequency to 640Hz with a timer clock of 8MHz */
static uint32_t pulse_value4 = 6250;

/***********************************************************************************************************/
/*                                       Static Function Prototypes                                        */
/***********************************************************************************************************/

/**
 * @brief Function for configuring the system clock and the LSE as MCO1 output.
 * @return void.
 */
static void SetMCO_LSE(void);

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void Timer6_Config(void){

    Timer.tim_num = TIMER6;
    Timer.pTimer = TIM6;
    Timer.prescaler = 240;
    Timer.period = 64000 - 1;

    Timer_Init(&Timer);
    Timer_IRQConfig(IRQ_NO_TIM6_DAC, ENABLE);
    Timer_Start(&Timer);
}

void Timer2_Config(void){

    IC_Handle_t IC = {0};
    GPIO_Handle_t GpioIC = {0};

    /* Configure system clock and LSE for testing */
    SetMCO_LSE();

    /* Configure the basic timer function */
    Timer.tim_num = TIMER2;
    Timer.pTimer = TIM2;
    Timer.prescaler = 0;
    Timer.period = 0xFFFFFFFF;  /* Set timer for counting until its maximum capacity (32 bits) */
    Timer_Init(&Timer);

    /* Configure the input capture function */
    IC.ic_polarity = CC_POLARITY_RISING;
    IC.ic_select = CC_IN_TI1;
    IC.ic_prescaler = IC_NO_PRESCALER;
    IC.ic_filter = 0;
    Timer_ICInit(&Timer, IC, CHANNEL1);

    /* Configure the GPIO */
    GpioIC.pGPIOx = GPIOA;
    GpioIC.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GpioIC.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    GpioIC.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GpioIC.GPIO_PinConfig.GPIO_PinAltFunMode = 1;
    GPIO_Init(&GpioIC);

    /* Enable interrupt */
    Timer_IRQConfig(IRQ_NO_TIM2, ENABLE);
    /* Start timer */
    Timer_Start(&Timer);
}

void Timer2_Process(void){

    uint32_t capture_dif = 0;
    double timer_cnt_res = 0;
    double signal_period = 0;
    double timer_cnt_freq = 0;
    double signal_freq = 0;

    if(capture_done){
        if(input_capture[1] > input_capture[0]){
            capture_dif = input_capture[1] - input_capture[0];
        }
        else{
            capture_dif = (0xFFFFFFFF - input_capture[0]) + input_capture[1];
        }

        /* If APB1 prescaler is not 1 the frequency need to be multiplied by 2 following reference manual */
        timer_cnt_freq = RCC_GetPCLK1Value()/(Timer.prescaler + 1);
        timer_cnt_res = 1/timer_cnt_freq;
        signal_period = capture_dif*timer_cnt_res;
        signal_freq = 1/signal_period;

        printf("Frequency: %f\r\n", signal_freq);

        capture_done = 0;
    }
}

void Timer4_Config(void){

    OC_Handle_t OC = {0};
    GPIO_Handle_t GpioOC = {0};
    RCC_Config_t RCC_Cfg = {0};

    /* Set configuration */
    RCC_Cfg.clk_source = RCC_CLK_SOURCE_HSE;
    RCC_Cfg.hse_mode = RCC_HSE_BYPASS;
    /* Set clock */
    RCC_SetSystemClock(RCC_Cfg);

    /* Configure the basic timer function */
    Timer.tim_num = TIMER4;
    Timer.pTimer = TIM4;
    Timer.prescaler = 0;
    Timer.period = 0xFFFF;  /* Set timer for counting until its maximum capacity (16 bits) */
    Timer_Init(&Timer);

    /* Configure the output compare function */
    OC.oc_mode = OC_MODE_TOGGLE;
    OC.oc_polarity = CC_POLARITY_RISING;
    OC.oc_preload = OC_PRELOAD_DISABLE;
    OC.oc_pulse = pulse_value1;
    Timer_OCInit(&Timer, OC, CHANNEL1);
    OC.oc_pulse = pulse_value2;
    Timer_OCInit(&Timer, OC, CHANNEL2);
    OC.oc_pulse = pulse_value3;
    Timer_OCInit(&Timer, OC, CHANNEL3);
    OC.oc_pulse = pulse_value4;
    Timer_OCInit(&Timer, OC, CHANNEL4);

    /* Configure the GPIO */
    GpioOC.pGPIOx = GPIOB;
    GpioOC.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GpioOC.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    GpioOC.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GpioOC.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULL;
    GpioOC.GPIO_PinConfig.GPIO_PinAltFunMode = 2;
    GpioOC.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GPIO_Init(&GpioOC);
    GpioOC.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Init(&GpioOC);
    GpioOC.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
    GPIO_Init(&GpioOC);
    GpioOC.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    GPIO_Init(&GpioOC);

    /* Enable interrupt */
    Timer_IRQConfig(IRQ_NO_TIM4, ENABLE);
    /* Start timer */
    Timer_Start(&Timer);
}

void Timer3_Config(void){

    OC_Handle_t PWM = {0};
    GPIO_Handle_t GpioPWM = {0};
    RCC_Config_t RCC_Cfg = {0};

    /* Set configuration */
    RCC_Cfg.clk_source = RCC_CLK_SOURCE_HSE;
    RCC_Cfg.hse_mode = RCC_HSE_BYPASS;
    /* Set clock */
    RCC_SetSystemClock(RCC_Cfg);

    /* Configure the basic timer function */
    Timer.tim_num = TIMER3;
    Timer.pTimer = TIM3;
    Timer.prescaler = 0;
    Timer.period = 8000 - 1;  /* Set timer for counting 1ms */
    Timer_Init(&Timer);

    /* Configure the output compare function */
    PWM.oc_mode = OC_MODE_PWM1;
    PWM.oc_polarity = CC_POLARITY_RISING;
    PWM.oc_pulse = 0;
    PWM.oc_preload = OC_PRELOAD_ENABLE;
    Timer_OCInit(&Timer, PWM, CHANNEL1);

    /* Configure the GPIO */
    GpioPWM.pGPIOx = GPIOA;
    GpioPWM.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GpioPWM.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    GpioPWM.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GpioPWM.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULL;
    GpioPWM.GPIO_PinConfig.GPIO_PinAltFunMode = 2;
    GPIO_Init(&GpioPWM);

    /* Start timer */
    Timer_Start(&Timer);
}

void Timer3_Process(void){

    static uint16_t brightness = 0;

    while(brightness < Timer.period){
        brightness += 10;
        Timer_CCSetValue(&Timer, CHANNEL1, brightness);
        delay_ms(1);
    }
    while(brightness > 0){
        brightness -= 10;
        Timer_CCSetValue(&Timer, CHANNEL1, brightness);
        delay_ms(1);
    }
}

/***********************************************************************************************************/
/*                               Weak Function Overwrite Definitions                                       */
/***********************************************************************************************************/

void TIM6_DAC_Handler(void){
    Timer_IRQHandling(&Timer);
}

void TIM2_Handler(void){
    Timer_IRQHandling(&Timer);
}

void TIM4_Handler(void){
    Timer_IRQHandling(&Timer);
}

void Timer_ApplicationEventCallback(Timer_Num_t tim_num, Timer_Event_t timer_event){

    static uint8_t count = 1;
    uint32_t ccr_content = 0;

    if(timer_event == TIMER_UIF_EVENT){
        if(tim_num == TIMER6){
            GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
        }
    }
    else if(timer_event == TIMER_CC1IF_EVENT){
        if(tim_num == TIMER2){
            if(!capture_done){
                if(count == 1){
                    input_capture[0] = Timer_CCGetValue(&Timer, CHANNEL1);
                    count++;
                }
                else if(count == 2){
                    input_capture[1] = Timer_CCGetValue(&Timer, CHANNEL1);
                    count = 1;
                    capture_done = 1;
                }
            }
        }
        else if(tim_num == TIMER4){
            ccr_content = Timer_CCGetValue(&Timer, CHANNEL1);
            Timer_CCSetValue(&Timer, CHANNEL1, ccr_content + pulse_value1);
        }
        else{
            /* do nothing */
        }
    }
    else if(timer_event == TIMER_CC2IF_EVENT){
        if(tim_num == TIMER4){
            ccr_content = Timer_CCGetValue(&Timer, CHANNEL2);
            Timer_CCSetValue(&Timer, CHANNEL2, ccr_content + pulse_value2);
        }
    }
    else if(timer_event == TIMER_CC3IF_EVENT){
        if(tim_num == TIMER4){
            ccr_content = Timer_CCGetValue(&Timer, CHANNEL3);
            Timer_CCSetValue(&Timer, CHANNEL3, ccr_content + pulse_value3);
        }
    }
    else if(timer_event == TIMER_CC4IF_EVENT){
        if(tim_num == TIMER4){
            ccr_content = Timer_CCGetValue(&Timer, CHANNEL4);
            Timer_CCSetValue(&Timer, CHANNEL4, ccr_content + pulse_value4);
        }
    }
    else{
        /* do nothing */
    }
}

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/

static void SetMCO_LSE(void){

    RCC_Config_t RCC_Cfg = {0};
    GPIO_Handle_t MCOPin = {0};

    /* Set configuration */
    RCC_Cfg.mco1_source = MCO1_LSE;
    RCC_Cfg.mco1_presc = MCO_P_1;
    RCC_Cfg.lse_bypass = RCC_LSE_CRYSTAL;
    /* Set MCO1 */
    RCC_SetMCO1Clk(RCC_Cfg);
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
}
