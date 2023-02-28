/********************************************************************************************************//**
* @file pwr_driver.h
*
* @brief File containing the APIs for configuring the power peripheral.
*
* Public Functions:
*       - void PWR_SetOverDrive(void)
*       - void PWR_UnsetOverDrive(void)
*       - void PWR_SetRegVoltageScal(PWR_RegVoltScal_t reg_volt_scal)
*       - void PWR_DisableBackupWrProtec(void)
*       - void PWR_EnableBackupWrProtec(void)
**/

#ifndef PWR_DRIVER_H
#define PWR_DRIVER_H

/**
 * @brief Possible options for the regulator voltage scaling output.
 */
typedef enum
{
    VOS_SCALE_3 = 1,
    VOS_SCALE_2 = 2,
    VOS_SCALE_1 = 3
}PWR_RegVoltScal_t;

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to set the over-drive mode.
 * @return void.
 */
void PWR_SetOverDrive(void);

/**
 * @brief Function to unset the over-drive mode.
 * @return void.
 */
void PWR_UnsetOverDrive(void);

/**
 * @brief Function to set the regulator voltage scaling output.
 * @param[in] reg_volt_scal is the voltage scaling option.
 * @return void.
 */
void PWR_SetRegVoltageScal(PWR_RegVoltScal_t reg_volt_scal);

/**
 * @brief Function to disable the backup domain write protection.
 * @return void.
 */
void PWR_DisableBackupWrProtec(void);

/**
 * @brief Function to enable the backup domain write protection.
 * @return void.
 */
void PWR_EnableBackupWrProtec(void);

#endif  /* PWR_DRIVER_H */