/********************************************************************************************************//**
* @file can_driver.h
*
* @brief Header file containing the prototypes of the APIs for configuring the CAN peripheral.
*
* Public Functions:
*       - void CAN_Init(CAN_Handle_t* pCAN_Handle)
*       - void CAN_DeInit(CAN_RegDef_t* pCANx)
*       - void CAN_PerClkCtrl(CAN_RegDef_t* pCANx, uint8_t en_or_di)
*/

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "stm32f446xx.h"
#include <stdint.h>

/**
 * @defgroup CAN_operation_mode CAN possible operation modes.
 * @{
 */
#define CAN_MODE_NORMAL                 0   /**< @brief Normal operation */
#define CAN_MODE_LOOPBACK               1   /**< @brief Loopback mode enable */
#define CAN_MODE_SILENT                 2   /**< @brief Silent mode */
#define CAN_MODE_SILENT_LOOPBACK        3   /**< @brief Loopback mode enable and silent mode */
/**@}*/

/**
 * @defgroup CAN_auto_bus_off automatic bus-off management
 * @{
 */
#define CAN_ABOM_DISABLE                0   /**< @brief The bus-off state is left on software request */
#define CAN_ABOM_ENABLE                 1   /**< @brief The bus-off state is left automatically */
/**@}*/

/**
 * @defgroup CAN_auto_retransmission no automatic retransmission
 * @{
 */
#define CAN_NART_ON                     0   /**< @brief The CAN hardware automatically retransmits the message */
#define CAN_NART_OFF                    1   /**< @brief A message is transmitted only once */
/**@}*/

/**
 * @defgroup CAN_auto_wakeup automatic wakeup mode
 * @{
 */
#define CAN_AUTO_WAKEUP_OFF             0   /**< @brief The sleep mode is left on software request */
#define CAN_AUTO_WAKEUP_ON              1   /**< @brief The sleep mode is left automatically by hardware */
/**@}*/

/**
 * @defgroup CAN_receive_FIFO_locked receive FIFO locked mode
 * @{
 */
#define CAN_RX_FIFO_NOT_LOCKED          0   /**< @brief The receive FIFO not locked on overrun */
#define CAN_RX_FIFO_LOCKED              1   /**< @brief The receive FIFO locked against overrun */
/**@}*/

/**
 * @defgroup CAN_time_trigger_mode time triggered communication mode
 * @{
 */
#define CAN_TTCM_DISABLE                0   /**< @brief Time triggered communication mode disabled */
#define CAN_TTCM_ENABLE                 1   /**< @brief Time triggered communication mode enabled */
/**@}*/

/**
 * @defgroup CAN_tx_FIFO_priority transmit FIFO priority
 * @{
 */
#define CAN_TXFP_ID                     0   /**< @brief Priority driven by the id of the message */
#define CAN_TXFP_REQUEST                1   /**< @brief Priority driven by the request order */
/**@}*/

/**
 * @brief Handle structure for CAN peripheral.
 */
typedef struct
{
    uint8_t CAN_Mode;                   /**< @brief Possible values of @ref CAN_operation_mode */
    uint8_t CAN_AutoBusOff;             /**< @brief Possible values of @ref CAN_auto_bus_off */
    uint8_t CAN_AutoRetransmission;     /**< @brief Possible values of @ref CAN_auto_retransmission */
    uint8_t CAN_AutoWakeup;             /**< @brief Possible values of @ref CAN_auto_wakeup */
    uint8_t CAN_ReceiveFifoLocked;      /**< @brief Possible values of @ref CAN_receive_FIFO_locked */
    uint8_t CAN_TimeTriggerMode;        /**< @brief Possible values of @ref CAN_time_trigger_mode */
    uint8_t CAN_TxFifoPriority;         /**< @brief Possible values of @ref CAN_tx_FIFO_priority */
    uint16_t CAN_Prescalar;             /**< @brief Baud rate prescaler */
    uint8_t CAN_SyncJumpWidth;          /**< @brief Resynchronization jump width */
    uint8_t CAN_TimeSeg1;               /**< @brief Time segment 1 */
    uint8_t CAN_TimeSeg2;               /**< @brief Time segment 2 */
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
void CAN_DeInit(CAN_RegDef_t* pCANx);
void CAN_PerClkCtrl(CAN_RegDef_t* pCANx, uint8_t en_or_di);

#endif /* CAN_DRIVER_H */
