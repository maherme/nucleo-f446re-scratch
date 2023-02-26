/********************************************************************************************************//**
* @file pwr_driver.c
*
* @brief File containing the APIs for configuring the power peripheral.
*
* Public Functions:
*       - void PWR_SetOverDrive(void)
*       - void PWR_UnsetOverDrive(void)
*       - void PWR_SetRegVoltageScal(PWR_RegVoltScal_t reg_volt_scal)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include "pwr_driver.h"
#include "stm32f446xx.h"

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