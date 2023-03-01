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
*       - void PWR_EnterStopMode(PWR_StopMode_t stop_mode, PWR_SleepMode_t sleep_mode)
**/

#ifndef PWR_DRIVER_H
#define PWR_DRIVER_H

#include <stdint.h>

/**
 * @brief Possible options for the regulator voltage scaling output.
 */
typedef enum
{
    VOS_SCALE_3 = 1,
    VOS_SCALE_2 = 2,
    VOS_SCALE_1 = 3
}PWR_RegVoltScal_t;

/**
 * @brief Possible options for the wake up pin.
 */
typedef enum
{
    PWR_WKUP1 = 0x00000100,
    PWR_WKUP2 = 0x00000080
}PWR_WakeupPin_t;

/**
 * @brief Possible options for entering in stop mode.
 */
typedef enum
{
    PWR_STOP_MR,        /**< @brief Main regulator ON and flash ON */
    PWR_STOP_MR_FPD,    /**< @brief Main regulator ON and flash OFF */
    PWR_STOP_LP,        /**< @brief Low power regulator ON and flash ON */
    PWR_STOP_LP_FPD,    /**< @brief Low power regulator ON and flash OFF */
    PWR_STOP_UMR_FPD,   /**< @brief Main regulator ON in under drive mode */
    PWR_STOP_ULP_FPD,   /**< @brief Low power regulator ON in under drive mode */
}PWR_StopMode_t;

/**
 * @brief Possible options for low power when entering in stop mode.
 */
typedef enum
{
    PWR_STOP_WFI,   /**< @brief Wake for interrupt low power mode */
    PWR_STOP_WFE    /**< @brief Wafe for event low power mode */
}PWR_SleepMode_t;

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

/**
 * @brief Function to enter in standby mode.
 * @return void.
 */
void PWR_EnterSTANDBY(void);

/**
 * @brief Function to enable a wake up pin.
 * @param[in] pin is the wake up pin selected.
 * @return 0 is OK.
 * @return 1 is pin selected is not correct.
 */
uint8_t PWR_EnableWakeupPin(PWR_WakeupPin_t pin);

/**
 * @brief Function to disable a wake up pin.
 * @param[in] pin is the wake up pin selected.
 * @return 0 is OK.
 * @return 1 is pin selected is not correct.
 */
uint8_t PWR_DisableWakeupPin(PWR_WakeupPin_t pin);

/**
 * @brief Function to check if device has been in standby mode.
 * @return 0 is device has not been in standby mode.
 * @return 1 is device has been in standby mode.
 */
uint8_t PWR_CheckWakeupStandby(void);

/**
 * @brief Function to clear the wake up flag (WUF).
 * @return void.
 */
void PWR_ClearWakeupFlag(void);

/**
 * @brief Function to clear the standby flag (SBF).
 * @return void.
 */
void PWR_ClearStandbyFlag(void);

/**
 * @brief Function to enable the back up regulator.
 * @return void.
 */
void PWR_EnableBackupRegulator(void);

/**
 * @brief Function to disable the back up regulator.
 * @return void.
 */
void PWR_DisableBackupRegulator(void);

/**
 * @brief Function to configure the FLASH power down in stop mode.
 * @param[in] en_or_di for enable or disable.
 * @return void.
 */
void PWR_EnableFLASHPowerDown(uint8_t en_or_di);

/**
 * @brief Function to configure the under drive in stop mode.
 * @param[in] en_or_di for enable or disable.
 * @return void.
 */
void PWR_EnableUnderDrive(uint8_t en_or_di);

/**
 * @brief Function to configure the main regulator in deepsleep under drive mode.
 * @param[in] en_or_di for enable or disable.
 * @return void.
 */
void PWR_EnableMRUnderDrive(uint8_t en_or_di);

/**
 * @brief Function to configure the low power regulator in deepsleep under drive mode.
 * @param[in] en_or_di for enable or disable.
 * @return void.
 */
void PWR_EnableLPRUnderDrive(uint8_t en_or_di);

/**
 * @brief Function to enter in low power stop mode.
 * @param[in] stop_mode for selecting the stop operating modes.
 * @param[in] sleep_mode for selecting the cortex low power mode.
 * @return void.
 */
void PWR_EnterStopMode(PWR_StopMode_t stop_mode, PWR_SleepMode_t sleep_mode);

#endif  /* PWR_DRIVER_H */