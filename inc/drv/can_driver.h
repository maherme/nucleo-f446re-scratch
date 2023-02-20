/********************************************************************************************************//**
* @file can_driver.h
*
* @brief Header file containing the prototypes of the APIs for configuring the CAN peripheral.
*
* Public Functions:
*       - uint8_t CAN_Init(CAN_Handle_t* pCAN_Handle)
*       - void CAN_DeInit(CAN_RegDef_t* pCANx)
*       - void CAN_PerClkCtrl(CAN_RegDef_t* pCANx, uint8_t en_or_di)
*       - uint8_t CAN_AddTxMsg(CAN_RegDef_t* pCANx, CAN_TxHeader_t* pTxHeader, uint8_t* msg, uint32_t mailbox)
*       - uint8_t CAN_TxMsgPending(CAN_RegDef_t* pCANx, uint32_t mailbox)
*       - uint8_t CAN_SetFilter(CAN_Filter_t* filter)
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
 * @defgroup CAN_id_extension defines the identifier type of message in the mailbox
 * @{
 */
#define CAN_STDI                        0   /**< @brief Standard identifier */
#define CAN_EXID                        1   /**< @brief Extended identifier */
/**@}*/

/**
 * @defgroup CAN_remote_tx_request remote transmission request
 * @{
 */
#define CAN_DATA_FRAME                  0   /**< @brief Data frame */
#define CAN_REMOTE_FRAME                1   /**< @brief Remote frame */
/**@}*/

/**
 * @defgroup CAN_mailboxes mailboxes for CAN
 * @{
 */
#define CAN_MAILBOX_0                   (uint32_t)0x00000001    /**< @brief Mailbox 0 */
#define CAN_MAILBOX_1                   (uint32_t)0x00000002    /**< @brief Mailbox 1 */
#define CAN_MAILBOX_2                   (uint32_t)0x00000004    /**< @brief Mailbox 2 */
/**@}*/

/**
 * @defgroup CAN_filter_mode filter mode
 * @{
 */
#define CAN_FILTER_ID_MASK_MODE         0   /**< @brief Bits of filter bank x are in Identifier Mask mode */
#define CAN_FILTER_ID_LIST_MODE         1   /**< @brief Bits of filter bank x are in Identifier List mode */
/**@}*/

/**
 * @defgroup CAN_filter_scale filter scale configuration
 * @{
 */
#define CAN_FILTER_16_BIT_SCALE         0   /**< @brief Dual 16-bit scale configuration */
#define CAN_FILTER_32_BIT_SCALE         1   /**< @brief Single 32-bit scale configuration */
/**@}*/

/**
 * @defgroup CAN_filter_FIFO filter FIFO assignment
 * @{
 */
#define CAN_FILTER_FIFO_0               0   /**< @brief Filter assigned to FIFO 0 */
#define CAN_FILTER_FIFO_1               1   /**< @brief Filter assigned to FIFO 1 */
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
    CAN_RegDef_t* pCANx;            /**< @brief Base address of the DMAx peripheral */
    CAN_Config_t CAN_Config;        /**< @brief DMAx peripheral configuration settings */
}CAN_Handle_t;

/**
 * @brief Structure for CAN Tx message header.
 */
typedef struct
{
    uint16_t StId;                  /**< @brief Standard identifier */
    uint32_t ExId;                  /**< @brief Extended identifier */
    uint8_t IDE;                    /**< @brief Possible values of @ref CAN_id_extension */
    uint8_t RTR;                    /**< @brief Possible values of @ref CAN_remote_tx_request */
    uint8_t DLC;                    /**< @brief Data length code */
}CAN_TxHeader_t;

typedef struct 
{
    uint8_t FilterNumber;           /**< @brief Filter number, it must be a value from 0 to 27 */
    uint8_t Mode;                   /**< @brief Filter mode, possible values of @ref CAN_filter_mode */
    uint8_t Scale;                  /**< @brief Filter scale, possible values of @ref CAN_filter_scale */
    uint8_t FIFO;                   /**< @brief FIFO assignment, possible values of @ref CAN_filter_FIFO */
    uint16_t IdentifierLR;          /**< @brief Low half word of identifier register filter */
    uint16_t IdentifierHR;          /**< @brief High half word of identifier register filter */
    uint16_t MaskLR;                /**< @brief Low half word of mask register filter */
    uint16_t MaskHR;                /**< @brief High half word of mask register filter */
}CAN_Filter_t;

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to initialize CAN peripheral.
 * @param[in] pCAN_Handle handle structure for the CAN peripheral.
 * @return 0 is the initialization process was OK
 * @return 1 is the timing paratemers are not correct
 */
uint8_t CAN_Init(CAN_Handle_t* pCAN_Handle);

/**
 * @brief Function to reset all register of a CAN peripheral.
 * @param[in] pCANx the base address of the CANx peripheral.
 * @return void
 */
void CAN_DeInit(CAN_RegDef_t* pCANx);

/**
 * @brief Function to control the peripheral clock of the CAN peripheral.
 * @param[in] pCANx the base address of the CANx peripheral.
 * @param[in] en_or_di for enable or disable.
 * @return void
 */
void CAN_PerClkCtrl(CAN_RegDef_t* pCANx, uint8_t en_or_di);

/**
 * @brief Function to add a message to a transmission mailbox.
 * @param[in] pCANx the base address of the CANx peripheral.
 * @param[in] pTxHeader struct with information about the message header.
 * @param[in] msg is a pointer to the information to be sent.
 * @param[in] mailbox the selected mailbox: possible value of @ref CAN_mailboxes.
 * @return 0 is adding message was OK.
 * @return 1 is an invalid mailbox value is passed as parameter.
 * @return 2 is the selected mailbox is in transmission pending state.
 */
uint8_t CAN_AddTxMsg(CAN_RegDef_t* pCANx, CAN_TxHeader_t* pTxHeader, uint8_t* msg, uint32_t mailbox);

/**
 * @brief Function to check if a transmission request is pending in a mailbox.
 * @param[in] pCANx the base address of the CANx peripheral.
 * @param[in] mailbox the selected mailbox: possible value of @ref CAN_mailboxes.
 * @return 0 if no transmission is pending.
 * @return 1 if a transmission is pending.
 */
uint8_t CAN_TxMsgPending(CAN_RegDef_t* pCANx, uint32_t mailbox);

/**
 * @brief Function to configure the CAN repection filters.
 * @param[in] filter struct with information about the filter configuration parameters.
 * @return 0 if the filter was configured.
 * @return 1 if the selected filter number is not correct.
 */
uint8_t CAN_SetFilter(CAN_Filter_t* filter);

#endif /* CAN_DRIVER_H */
