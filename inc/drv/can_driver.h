/********************************************************************************************************//**
* @file can_driver.h
*
* @brief Header file containing the prototypes of the APIs for configuring the CAN peripheral.
*
* Public Functions:
*/

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "stm32f446xx.h"
#include <stdint.h>

/**
 * @brief Handle structure for CAN peripheral.
 */
typedef struct
{
    uint8_t CAN_Mode;
    uint8_t CAN_AutoBusOff;
    uint8_t CAN_AutoRetransmission;
    uint8_t CAN_AutoWakeup;
    uint8_t CAN_ReceiveFifoLocked;
    uint8_t CAN_TimeTriggerMode;
    uint8_t CAN_TxFifoPriority;
    uint16_t CAN_Prescalar;
    uint8_t CAN_SyncJumpWidth;
    uint8_t CAN_TimeSeg1;
    uint8_t CAN_TimeSeg2;
}CAN_Config_t;

/**
 * @brief Handle structure for CAN peripheral.
 */
typedef struct
{
    CAN_RegDef_t* pCANx;            /**< Base address of the DMAx peripheral */
    CAN_Config_t CAN_Config;        /**< DMAx peripheral configuration settings */
}CAN_Handle_t;

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

void CAN_Init(CAN_Handle_t* pCAN_Handle);

#endif /* CAN_DRIVER_H */
