/********************************************************************************************************//**
* @file pwr_driver.c
*
* @brief File containing the APIs for configuring the power peripheral.
*
* Public Functions:
*       - void PWR_SetOverDrive(void)
*       - void PWR_UnsetOverDrive(void)
*       - void PWR_SetRegVoltageScal(PWR_RegVoltScal_t reg_volt_scal)
*       - void PWR_DisableBackupWrProtec(void)
*       - void PWR_EnableBackupWrProtec(void)
*       - void PWR_EnterSTANDBY(void)
*       - uint8_t PWR_EnableWakeupPin(PWR_WakeupPin_t pin)
*       - uint8_t PWR_DisableWakeupPin(PWR_WakeupPin_t pin)
*       - uint8_t PWR_CheckWakeupStandby(void)
*       - void PWR_ClearWakeupFlag(void)
*       - void PWR_ClearStandbyFlag(void)
*       - void PWR_EnableBackupRegulator(void)
*       - void PWR_DisableBackupRegulator(void)
*       - void PWR_EnableFLASHPowerDown(uint8_t en_or_di)
*       - void PWR_EnableUnderDrive(uint8_t en_or_di)
*       - void PWR_EnableMRUnderDrive(uint8_t en_or_di)
*       - void PWR_EnableLPRUnderDrive(uint8_t en_or_di)
*       - void PWR_EnterStopMode(PWR_StopMode_t stop_mode, PWR_SleepMode_t sleep_mode)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include "pwr_driver.h"
#include "cortex_m4.h"
#include "stm32f446xx.h"
#include <stdint.h>

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void PWR_SetOverDrive(void){

    /* Set over-drive enable bit */
    PWR->CR |= (1 << PWR_CR_ODEN);
    /* Wait until over-drive mode ready */
    while(!(PWR->CSR & (1 << PWR_CSR_ODRDY)));
    /* Set over-driver switching bit */
    PWR->CR |= (1 << PWR_CR_ODSWEN);
    /* Wait until over drive mode is active */
    while(!(PWR->CSR & (1 << PWR_CSR_ODSWRDY)));
}

void PWR_UnsetOverDrive(void){

    uint32_t temp = 0;

    /* Reset over-drive enable and over-drive switching bit */
    temp |= ((1 << PWR_CR_ODEN) |
            (1 << PWR_CR_ODSWEN));
    PWR->CR &= ~temp;
    /* Wait unitl over drive mode is inactive */
    while(PWR->CSR & (1 << PWR_CSR_ODSWRDY));
}

void PWR_SetRegVoltageScal(PWR_RegVoltScal_t reg_volt_scal){

    PWR->CR &= ~(0x2 << PWR_CR_VOS);
    PWR->CR |= (reg_volt_scal << PWR_CR_VOS);
}

void PWR_DisableBackupWrProtec(void){

    PWR->CR |= (1 << PWR_CR_DBP);
}

void PWR_EnableBackupWrProtec(void){

    PWR->CR &= ~(1 << PWR_CR_DBP);
}

void PWR_EnterSTANDBY(void){

    PWR->CR |= (1 << PWR_CR_PDDS);
    EnableSleepDeep();
    Enter_WFI();
}

uint8_t PWR_EnableWakeupPin(PWR_WakeupPin_t pin){

    uint32_t temp = 0;

    if(pin & 0xFFFFFE7F){
        return 1;
    }

    temp |= pin;
    PWR->CSR |= temp;

    return 0;
}

uint8_t PWR_DisableWakeupPin(PWR_WakeupPin_t pin){

    uint32_t temp = 0;

    if(pin & 0xFFFFFE7F){
        return 1;
    }

    temp |= pin;
    PWR->CSR &= ~temp;

    return 0;
}

uint8_t PWR_CheckWakeupStandby(void){

    if(PWR->CSR & (1 << PWR_CSR_SBF)){
        return 1;
    }
    else{
        return 0;
    }
}

void PWR_ClearWakeupFlag(void){

    PWR->CR |= (1 << PWR_CR_CWUF);
}

void PWR_ClearStandbyFlag(void){

    PWR->CR |= (1 << PWR_CR_CSBF);
}

void PWR_EnableBackupRegulator(void){

    PWR->CSR |= (1 << PWR_CSR_BRE);
    while(!(PWR->CSR & (1 << PWR_CSR_BRR)));
}

void PWR_DisableBackupRegulator(void){

    PWR->CSR &= ~(1 << PWR_CSR_BRE);
    while(PWR->CSR & (1 << PWR_CSR_BRR));
}

void PWR_EnableFLASHPowerDown(uint8_t en_or_di){

    if(en_or_di == ENABLE){
        PWR->CR |= (1 << PWR_CR_FPDS);
    }
    else{
        PWR->CR &= ~(1 << PWR_CR_FPDS);
    }
}

void PWR_EnableUnderDrive(uint8_t en_or_di){

    if(en_or_di == ENABLE){
        PWR->CR |= (0x3 << PWR_CR_UDEN);
    }
    else{
        PWR->CR &= ~(0x3 << PWR_CR_UDEN);
    }
}

void PWR_EnableMRUnderDrive(uint8_t en_or_di){

    if(en_or_di == ENABLE){
        PWR->CR |= (1 << PWR_CR_MRUDS);
    }
    else{
        PWR->CR &= ~(1 << PWR_CR_MRUDS);
    }
}

void PWR_EnableLPRUnderDrive(uint8_t en_or_di){

    if(en_or_di == ENABLE){
        PWR->CR |= (1 << PWR_CR_LPUDS);
    }
    else{
        PWR->CR &= ~(1 << PWR_CR_LPUDS);
    }
}

void PWR_EnterStopMode(PWR_StopMode_t stop_mode, PWR_SleepMode_t sleep_mode){

    /* Clear all bits related with the stop mode configuration*/
    PWR->CR &= ~0x000C0E01;

    /* Configure bits in the PWR control register */
    switch(stop_mode){
        case PWR_STOP_MR:
            /* MRUDS = 0, LPDS = 0, FPDS = 0 */
            break;
        case PWR_STOP_MR_FPD:
            /* MRUDS = 0, LPDS = 0, FPDS = 1 */
            PWR->CR |= (1 << PWR_CR_FPDS);
            break;
        case PWR_STOP_LP:
            /* MRUDS = 0, LPUDS = 0, LPDS = 1, FPDS = 0 */
            PWR->CR |= (1 << PWR_CR_LPDS);
            break;
        case PWR_STOP_LP_FPD:
            /* MRUDS = 0, LPUDS = 0, LPDS = 1, FPDS = 1 */
            PWR->CR |= (1 << PWR_CR_FPDS);
            PWR->CR |= (1 << PWR_CR_LPDS);
            break;
        case PWR_STOP_UMR_FPD:
            /* UDEN = 3, MRUDS = 1, LPDS = 0 */
            PWR->CR |= (0x3 << PWR_CR_UDEN);
            PWR->CR |= (1 << PWR_CR_MRUDS);
            break;
        case PWR_STOP_ULP_FPD:
            /* UDEN = 3, LPUDS = 1, LPDS = 1 */
            PWR->CR |= (0x3 << PWR_CR_UDEN);
            PWR->CR |= (1 << PWR_CR_LPUDS);
            PWR->CR |= (1 << PWR_CR_LPDS);
            break;
        default:
            /* Do nothing */
            break;
    }

    /* Enter in stop mode */
    EnableSleepDeep();
    if(sleep_mode == PWR_STOP_WFE){
        Enter_WFE();
    }
    else{
        Enter_WFI();
    }
}