/********************************************************************************************************//**
* @file i2c_driver.h
*
* @brief File containing the APIs for configuring the I2C peripheral.
*
* Public Functions:
*   - void    I2C_Init(I2C_Handle_t* pI2C_Handle)
*   - void    I2C_DeInit(I2C_RegDef_t* pI2Cx)
*   - void    I2C_PerClkCtrl(I2C_RegDef_t* pI2Cx, uint8_t en_or_di)
*   - void    I2C_MasterSendData(I2C_Handle_t* pI2C_Handle,
*                                uint8_t* pTxBuffer,
*                                uint32_t len,
*                                uint8_t slave_addr,
*                                sr_t sr)
*   - void    I2C_MasterReceiveData(I2C_Handle_t* pI2C_Handle,
*                                   uint8_t* pRxBuffer,
*                                   uint8_t len,
*                                   uint8_t slave_addr,
*                                   sr_t sr)
*   - uint8_t I2C_MasterSendDataIT(I2C_Handle_t* pI2C_Handle,
*                                  uint8_t* pTxBuffer,
*                                  uint32_t len,
*                                  uint8_t slave_addr,
*                                  sr_t sr)
*   - uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* pI2C_Handle,
*                                     uint8_t* pRxBuffer,
*                                     uint8_t len,
*                                     uint8_t slave_addr,
*                                     sr_t sr)
*   - void    I2C_SlaveSendData(I2C_RegDef_t* pI2Cx, uint8_t data)
*   - uint8_t I2C_SlaveReceiveData(I2C_RegDef_t* pI2Cx)
*   - void    I2C_EV_IRQHandling(I2C_Handle_t* pI2C_Handle)
*   - void    I2C_ER_IRQHandling(I2C_Handle_t* pI2C_Handle)
*   - void    I2C_Enable(I2C_RegDef_t* pI2Cx, uint8_t en_or_di)
*   - uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t flagname)
*   - void    I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx)
*   - void    I2C_ManageAcking(I2C_RegDef_t* pI2Cx, uint8_t en_or_di)
*   - void    I2C_SlaveEnCallbackEvents(I2C_RegDef_t* pI2Cx, uint8_t en_or_di)
*   - void    I2C_CloseReceiveData(I2C_Handle_t* pI2C_Handle)
*   - void    I2C_CloseSendData(I2C_Handle_t* pI2C_Handle)
*   - void    I2C_ApplicationEventCallback(I2C_Handle_t* pI2C_Handle, uint8_t app_event)
**/

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <stdint.h>
#include "stm32f446xx.h"

/**
 * @defgroup I2C_Speed I2C possible SCL speed values.
 * @{
 */
#define I2C_SCL_SPEED_SM        100000  /**< @brief Standard speed of 100KHz */
#define I2C_SCL_SPEED_FM2K      200000  /**< @brief High speed of 200KHz */
#define I2C_SCL_SPEED_FM4K      400000  /**< @brief Fast speed of 400KHz */
/**@}*/

/**
 * @defgroup I2C_Ack I2C possible ack control values.
 * @{
 */
#define I2C_ACK_ENABLE          1   /**< @brief I2C Acknowledge enable */
#define I2C_ACK_DISABLE         0   /**< @brief I2C Acknowledge disable */
/**@}*/

/**
 * @defgroup I2C_Duty I2C possible duty cycle values.
 * @{
 */
#define I2C_FM_DUTY_2           0   /**< @brief Fm mode Tlow/Thigh = 2 */
#define I2C_FM_DUTY_16_9        1   /**< @brief Fm mode Tlow/Thigh = 16/9 */
/**@}*/

/**
 * @defgroup I2C_Event I2C possible application states.
 * @{
 */
#define I2C_READY               0   /**< @brief I2C Ready */
#define I2C_BUSY_IN_RX          1   /**< @brief I2C Busy in reception */
#define I2C_BUSY_IN_TX          2   /**< @brief I2C Busy in transmission */
/**@}*/

/**
 * @defgroup I2C_Status I2C related status flags definitions.
 * @{
 */
#define I2C_FLAG_TXE        (1 << I2C_SR1_TXE)      /**< @brief TXE of I2C status register */
#define I2C_FLAG_RXNE       (1 << I2C_SR1_RXNE)     /**< @brief RXNE of I2C status register */
#define I2C_FLAG_SB         (1 << I2C_SR1_SB)       /**< @brief SB of I2C status register */
#define I2C_FLAG_OVR        (1 << I2C_SR1_OVR)      /**< @brief OVR of I2C status register */
#define I2C_FLAG_AF         (1 << I2C_SR1_AF)       /**< @brief AF of I2C status register */
#define I2C_FLAG_ARLO       (1 << I2C_SR1_ARLO)     /**< @brief ARLO of I2C status register */
#define I2C_FLAG_BERR       (1 << I2C_SR1_BERR)     /**< @brief BERR of I2C status register */
#define I2C_FLAG_STOPF      (1 << I2C_SR1_STOPF)    /**< @brief STOPF of I2C status register */
#define I2C_FLAG_ADD10      (1 << I2C_SR1_ADD10)    /**< @brief ADD10 of I2C status register */
#define I2C_FLAG_BTF        (1 << I2C_SR1_BTF)      /**< @brief BTF of I2C status register */
#define I2C_FLAG_ADDR       (1 << I2C_SR1_ADDR)     /**< @brief ADDR of I2C status register */
#define I2C_FLAG_TIMEOUT    (1 << I2C_SR1_TIMEOUT)  /**< @brief TIMEOUT of I2C status register */
/**@}*/

/**
 * @brief Possible options for rw in I2C_ExecuteAddressPhase().
 */
typedef enum{
    READ,   /**< Read selection */
    WRITE   /**< Write selection */
}rw_t;

/**
 * @brief Possible options for enable / disable for start repeating.
 */
typedef enum{
    I2C_DISABLE_SR, /**< Start repeating disable */
    I2C_ENABLE_SR   /**< Start repeating enable */
}sr_t;

/**
 * @defgroup I2C_AppEvent I2C possible application events
 * @{
 */
#define I2C_EVENT_TX_CMPLT  1   /**< @brief Transmission completed event */
#define I2C_EVENT_RX_CMPLT  2   /**< @brief Reception completed event */
#define I2C_EVENT_STOP      3   /**< @brief Stop event */
#define I2C_ERROR_BERR      4   /**< @brief Bus error event */
#define I2C_ERROR_ARLO      5   /**< @brief Arbitration lost event */
#define I2C_ERROR_AF        6   /**< @brief Acknowledge failure event */
#define I2C_ERROR_OVR       7   /**< @brief Overrun/underrun error event */
#define I2C_ERROR_TIMEOUT   8   /**< @brief Timeout error event */
#define I2C_EVENT_DATA_REQ  9   /**< @brief Requested data event */
#define I2C_EVENT_DATA_RCV  10  /**< @brief Received data event */
/**@}*/

/**
 * @brief Configuration structure for I2C peripheral.
 */
typedef struct
{
    uint32_t I2C_SCLSpeed;          /**< Possible values from @ref I2C_Speed */
    uint8_t I2C_DeviceAddress;      /**< Device Address */
    uint8_t I2C_ACKControl;         /**< Possible values from @ref I2C_Ack */
    uint16_t I2C_FMDutyCycle;       /**< Possible values from @ref I2C_Duty */
}I2C_Config_t;

/**
 * @brief Handle structure for I2Cx peripheral.
 */
typedef struct
{
    I2C_RegDef_t* pI2Cx;        /**< Base address of the I2Cx peripheral */
    I2C_Config_t I2C_Config;    /**< I2Cx peripheral configuration settings */
    uint8_t* pTxBuffer;         /**< To store the app. Tx buffer address */
    uint8_t* pRxBuffer;         /**< To store the app. Rx buffer address */
    uint32_t TxLen;             /**< To store Tx len */
    uint32_t RxLen;             /**< To store Rx len */
    uint8_t TxRxState;          /**< To store communication state */
    uint8_t DevAddr;            /**< To store slave / device address */
    uint32_t RxSize;            /**< To store Rx size */
    sr_t Sr;                    /**< To store repeated start value */
}I2C_Handle_t;

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to initialize I2C peripheral.
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 * @return void
 */
void I2C_Init(I2C_Handle_t* pI2C_Handle);

/**
 * @brief Function to reset all register of a I2C peripheral.
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @return void
 */
void I2C_DeInit(I2C_RegDef_t* pI2Cx);

/**
 * @brief Function to control the peripheral clock of the I2C peripheral.
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @param[in] en_or_di for enable or disable.
 * @return void
 */
void I2C_PerClkCtrl(I2C_RegDef_t* pI2Cx, uint8_t en_or_di);

/**
 * @brief Function to send data through I2C peripheral.
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 * @param[in] pTxBuffer buffer to store data to be transmitted.
 * @param[in] len length of the transmission buffer.
 * @param[in] slave_addr slave address.
 * @param[in] sr for enabling start repeating, possible values.
 * @return void
 * @note blocking call.
 */
void I2C_MasterSendData(I2C_Handle_t* pI2C_Handle,
                        uint8_t* pTxBuffer,
                        uint32_t len,
                        uint8_t slave_addr,
                        sr_t sr);

/**
 * @brief Function to receive data through I2C peripheral.
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 * @param[out] pRxBuffer buffer to store received data.
 * @param[in] len length of the transmission buffer.
 * @param[in] slave_addr slave address.
 * @param[in] sr for enabling start repeating, possible values.
 * @return void
 * @note blocking call.
 */
void I2C_MasterReceiveData(I2C_Handle_t* pI2C_Handle,
                           uint8_t* pRxBuffer,
                           uint8_t len,
                           uint8_t slave_addr,
                           sr_t sr);

/**
 * @brief Function to send data through I2C peripheral using insterrupt.
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 * @param[in] pTxBuffer buffer to store data to be transmitted.
 * @param[in] len length of the transmission buffer.
 * @param[in] slave_addr slave address.
 * @param[in] sr for enabling start repeating, possible values.
 * @return application state.
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t* pI2C_Handle,
                             uint8_t* pTxBuffer,
                             uint32_t len,
                             uint8_t slave_addr,
                             sr_t sr);

/**
 * @brief Function to receive data through I2C peripheral using insterrupt.
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 * @param[out] pRxBuffer buffer to store received data.
 * @param[in] len length of the transmission buffer.
 * @param[in] slave_addr slave address.
 * @param[in] sr for enabling start repeating, possible values.
 * @return application state.
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* pI2C_Handle,
                                uint8_t* pRxBuffer,
                                uint8_t len,
                                uint8_t slave_addr,
                                sr_t sr);

/**
 * @brief Function for sending data in slave mode using the I2C peripheral.
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @param[in] data to be sent.
 * @return void.
 */
void I2C_SlaveSendData(I2C_RegDef_t* pI2Cx, uint8_t data);

/**
 * @brief Function for receiving data in slave mode using the I2C peripheral.
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @return received data.
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t* pI2Cx);

/**
 * @brief Function to manage event interrupt of the I2C peripheral.
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 * @return void
 */
void I2C_EV_IRQHandling(I2C_Handle_t* pI2C_Handle);

/**
 * @brief Function to manage error interrupt of the I2C peripheral.
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 * @return void
 */
void I2C_ER_IRQHandling(I2C_Handle_t* pI2C_Handle);

/**
 * @brief Function enable the I2C peripheral.
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @param[in] en_or_di for enable or disable.
 * @return void.
 */
void I2C_Enable(I2C_RegDef_t* pI2Cx, uint8_t en_or_di);

/**
 * @brief Function returns the status of a given flag.
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @param[in] flagname the name of the flag.
 * @return flag status: FLAG_SET or FLAG_RESET.
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t flagname);

/**
 * @brief Function for generating the stop condition in the I2C peripheral.
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @return void.
 */
void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx);

/**
 * @brief Function to enable or disable the acking.
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @param[in] en_or_di to enable or disable @ref I2C_Ack
 * @return void.
 */
void I2C_ManageAcking(I2C_RegDef_t* pI2Cx, uint8_t en_or_di);

/**
 * @brief Function for enabling the callback events of the I2C peripheral.
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @param[in] en_or_di for enable or disable.
 * @return void.
 * @note applicable only in slave mode.
 */
void I2C_SlaveEnCallbackEvents(I2C_RegDef_t* pI2Cx, uint8_t en_or_di);

/**
 * @brief Function to close reception of data.
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 * @return void
 */
void I2C_CloseReceiveData(I2C_Handle_t* pI2C_Handle);

/**
 * @brief Function to close transmission of data.
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 * @return void
 */
void I2C_CloseSendData(I2C_Handle_t* pI2C_Handle);

/**
 * @brief Function for application callback.
 * @param[in] pI2C_Handle handle structure to I2C peripheral.
 * @param[in] app_event @ref I2C_AppEvent.
 * @return void.
 */
void I2C_ApplicationEventCallback(I2C_Handle_t* pI2C_Handle, uint8_t app_event);

#endif /* I2C_DRIVER_H */
