/*****************************************************************************************************
* FILENAME :        i2c_driver.h
*
* DESCRIPTION :
*       Header file containing the prototypes of the APIs for configuring the I2C peripheral.
*
* PUBLIC FUNCTIONS :
*       void    I2C_Init(I2C_Handle_t* pI2C_Handle)
*       void    I2C_DeInit(I2C_RegDef_t* pI2Cx)
*       void    I2C_PerClkCtrl(I2C_RegDef_t* pI2Cx, uint8_t en_or_di)
*       void    I2C_MasterSendData(I2C_Handle_t* pI2C_Handle, uint8_t* pTxBuffer, uint32_t len, uint8_t slave_addr, sr_t sr)
*       void    I2C_MasterReceiveData(I2C_Handle_t* pI2C_Handle, uint8_t* pRxBuffer, uint8_t len, uint8_t slave_addr, sr_t sr)
*       uint8_t I2C_MasterSendDataIT(I2C_Handle_t* pI2C_Handle, uint8_t* pTxBuffer, uint32_t len, uint8_t slave_addr, sr_t sr)
*       uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* pI2C_Handle, uint8_t* pRxBuffer, uint8_t len, uint8_t slave_addr, sr_t sr)
*       void    I2C_SlaveSendData(I2C_RegDef_t* pI2Cx, uint8_t data)
*       uint8_t I2C_SlaveReceiveData(I2C_RegDef_t* pI2Cx)
*       void    I2C_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       void    I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
*       void    I2C_EV_IRQHandling(I2C_Handle_t* pI2C_Handle)
*       void    I2C_ER_IRQHandling(I2C_Handle_t* pI2C_Handle)
*       void    I2C_Enable(I2C_RegDef_t* pI2Cx, uint8_t en_or_di)
*       uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t flagname)
*       void    I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx)
*       void    I2C_ManageAcking(I2C_RegDef_t* pI2Cx, uint8_t en_or_di)
*       void    I2C_SlaveEnCallbackEvents(I2C_RegDef_t* pI2Cx, uint8_t en_or_di)
*       void    I2C_CloseReceiveData(I2C_Handle_t* pI2C_Handle)
*       void    I2C_CloseSendData(I2C_Handle_t* pI2C_Handle)
*       void    I2C_ApplicationEventCallback(I2C_Handle_t* pI2C_Handle, uint8_t app_event)
*
**/

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <stdint.h>
#include "stm32f446xx.h"

/**
 * @I2C_SCLSPEED
 * I2C possible SCL speed values.
 */
#define I2C_SCL_SPEED_SM        100000
#define I2C_SCL_SPEED_FM2K      200000
#define I2C_SCL_SPEED_FM4K      400000

/**
 * @I2C_ACKCONTROL
 * I2C possible ack control values.
 */
#define I2C_ACK_ENABLE          1
#define I2C_ACK_DISABLE         0

/**
 * @I2C_FMDUTYCYCLE
 * I2C possible duty cycle values.
 */
#define I2C_FM_DUTY_2           0
#define I2C_FM_DUTY_16_9        1

/**
 * @I2C_APP_STATE
 * I2C possible application states.
 */
#define I2C_READY               0
#define I2C_BUSY_IN_RX          1
#define I2C_BUSY_IN_TX          2

/**
 * I2C related status flags definitions.
 */
#define I2C_FLAG_TXE        (1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE       (1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB         (1 << I2C_SR1_SB)
#define I2C_FLAG_OVR        (1 << I2C_SR1_OVR)
#define I2C_FLAG_AF         (1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO       (1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR       (1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF      (1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10      (1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF        (1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR       (1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT    (1 << I2C_SR1_TIMEOUT)

/**
 * @I2C_RW
 * Possible options for rw in I2C_ExecuteAddressPhase().
 */
typedef enum{
    READ,
    WRITE
}rw_t;

/**
 * @I2C_SR
 * Possible options for enable / disable for start repeating.
 */
typedef enum{
    I2C_DISABLE_SR,
    I2C_ENABLE_SR
}sr_t;

/**
 * I2C possible application events
 */
#define I2C_EVENT_TX_CMPLT  1
#define I2C_EVENT_RX_CMPLT  2
#define I2C_EVENT_STOP      3
#define I2C_ERROR_BERR      4
#define I2C_ERROR_ARLO      5
#define I2C_ERROR_AF        6
#define I2C_ERROR_OVR       7
#define I2C_ERROR_TIMEOUT   8
#define I2C_EVENT_DATA_REQ  9
#define I2C_EVENT_DATA_RCV  10

/**
 * Configuration structure for I2C peripheral.
 */
typedef struct
{
    uint32_t I2C_SCLSpeed;          /* Possible values from @I2C_SCLSPEED */
    uint8_t I2C_DeviceAddress;
    uint8_t I2C_ACKControl;         /* Possible values from @I2C_ACKCONTROL */
    uint16_t I2C_FMDutyCycle;       /* Possible values from @I2C_FMDUTYCYCLE */
}I2C_Config_t;

/**
 * Handle structure for I2Cx peripheral.
 */
typedef struct
{
    I2C_RegDef_t* pI2Cx;        /* Base address of the I2Cx peripheral */
    I2C_Config_t I2C_Config;    /* I2Cx peripheral configuration settings */
    uint8_t* pTxBuffer;         /* To store the app. Tx buffer address */
    uint8_t* pRxBuffer;         /* To store the app. Rx buffer address */
    uint32_t TxLen;             /* To store Tx len */
    uint32_t RxLen;             /* To store Rx len */
    uint8_t TxRxState;          /* To store communication state */
    uint8_t DevAddr;            /* To store slave / device address */
    uint32_t RxSize;            /* To store Rx size */
    sr_t Sr;                    /* To store repeated start value */
}I2C_Handle_t;

/*****************************************************************************************************/
/*                                       APIs Supported                                              */
/*****************************************************************************************************/

/**
 * @fn I2C_Init
 *
 * @brief function to initialize I2C peripheral.
 *
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 *
 * @return void
 */
void I2C_Init(I2C_Handle_t* pI2C_Handle);

/**
 *@fn I2C_DeInit
 *
 * @brief function to reset all register of a I2C peripheral.
 *
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 *
 * @return void
 */
void I2C_DeInit(I2C_RegDef_t* pI2Cx);

/**
 * @fn I2C_PerClkCtrl
 *
 * @brief function to control the peripheral clock of the I2C peripheral.
 *
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @param[in] en_or_di for enable or disable.
 *
 * @return void
 */
void I2C_PerClkCtrl(I2C_RegDef_t* pI2Cx, uint8_t en_or_di);

/**
 * @fn I2C_MasterSendData
 *
 * @brief function to send data through I2C peripheral.
 *
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 * @param[in] pTxBuffer buffer to store data to be transmitted.
 * @param[in] len length of the transmission buffer.
 * @param[in] slave_addr slave address.
 * @param[in] sr for enabling start repeating, possible values @I2C_SR.
 *
 * @return void
 *
 * @note blocking call.
 */
void I2C_MasterSendData(I2C_Handle_t* pI2C_Handle, uint8_t* pTxBuffer, uint32_t len, uint8_t slave_addr, sr_t sr);

/**
 * @fn I2C_MasterReceiveData
 *
 * @brief function to receive data through I2C peripheral.
 *
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 * @param[out] pRxBuffer buffer to store received data.
 * @param[in] len length of the transmission buffer.
 * @param[in] slave_addr slave address.
 * @param[in] sr for enabling start repeating, possible values @I2C_SR.
 *
 * @return void
 *
 * @note blocking call.
 */
void I2C_MasterReceiveData(I2C_Handle_t* pI2C_Handle, uint8_t* pRxBuffer, uint8_t len, uint8_t slave_addr, sr_t sr);

/**
 * @fn I2C_MasterSendDataIT
 *
 * @brief function to send data through I2C peripheral using insterrupt.
 *
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 * @param[in] pTxBuffer buffer to store data to be transmitted.
 * @param[in] len length of the transmission buffer.
 * @param[in] slave_addr slave address.
 * @param[in] sr for enabling start repeating, possible values @I2C_SR.
 *
 * @return application state.
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t* pI2C_Handle, uint8_t* pTxBuffer, uint32_t len, uint8_t slave_addr, sr_t sr);

/**
 * @fn I2C_MasterReceiveDataIT
 *
 * @brief function to receive data through I2C peripheral using insterrupt.
 *
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 * @param[out] pRxBuffer buffer to store received data.
 * @param[in] len length of the transmission buffer.
 * @param[in] slave_addr slave address.
 * @param[in] sr for enabling start repeating, possible values @I2C_SR.
 *
 * @return application state.
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* pI2C_Handle, uint8_t* pRxBuffer, uint8_t len, uint8_t slave_addr, sr_t sr);

/**
 * @fn I2C_SlaveSendData
 *
 * @brief function for sending data in slave mode using the I2C peripheral.
 *
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @param[in] data to be sent.
 *
 * @return void.
 */
void I2C_SlaveSendData(I2C_RegDef_t* pI2Cx, uint8_t data);

/**
 * @fn I2C_SlaveReceiveData
 *
 * @brief function for receiving data in slave mode using the I2C peripheral.
 *
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 *
 * @return received data.
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t* pI2Cx);

/**
 * @fn I2C_IRQConfig
 *
 * @brief function to configure the IRQ number of the I2C peripheral.
 *
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] en_or_di for enable or disable.
 *
 * @return void.
 */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di);

/**
 * @fn I2C_IRQConfig
 *
 * @brief function to configure the IRQ number of the I2C peripheral.
 *
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] IRQPriority priority of the interrupt.
 *
 * @return void.
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @fn I2C_EV_IRQHandling
 *
 * @brief function to manage event interrupt of the I2C peripheral.
 *
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 *
 * @return void
 */
void I2C_EV_IRQHandling(I2C_Handle_t* pI2C_Handle);

/**
 * @fn I2C_ER_IRQHandling
 *
 * @brief function to manage error interrupt of the I2C peripheral.
 *
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 *
 * @return void
 */
void I2C_ER_IRQHandling(I2C_Handle_t* pI2C_Handle);

/**
 * @fn I2C_Enable
 *
 * @brief function enable the I2C peripheral.
 *
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @param[in] en_or_di for enable or disable.
 *
 * @return void.
 */
void I2C_Enable(I2C_RegDef_t* pI2Cx, uint8_t en_or_di);

/**
 * @fn I2C_GetFlagStatus
 *
 * @brief function returns the status of a given flag.
 *
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @param[in] flagname the name of the flag.
 *
 * @return flag status: FLAG_SET or FLAG_RESET.
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t flagname);

/**
 * @fn I2C_GenerateStopCondition
 *
 * @brief function for generating the stop condition in the I2C peripheral.
 *
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 *
 * @return void.
 */
void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx);

/**
 * @fn I2C_ManageAcking
 *
 * @brief function to enable or disable the acking.
 *
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @param[in] en_or_di to enable or disable @I2C_ACKCONTROL
 *
 * @return void.
 */
void I2C_ManageAcking(I2C_RegDef_t* pI2Cx, uint8_t en_or_di);

/**
 * @fn I2C_SlaveEnCallbackEvents
 *
 * @brief function for enabling the callback events of the I2C peripheral.
 *
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @param[in] en_or_di for enable or disable.
 *
 * @return void.
 *
 * @note applicable only in slave mode.
 */
void I2C_SlaveEnCallbackEvents(I2C_RegDef_t* pI2Cx, uint8_t en_or_di);

/**
 * @fn I2C_CloseReceiveData
 *
 * @brief function to close reception of data.
 *
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 *
 * @return void
 */
void I2C_CloseReceiveData(I2C_Handle_t* pI2C_Handle);

/**
 * @fn I2C_CloseSendData
 *
 * @brief function to close transmission of data.
 *
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 *
 * @return void
 */
void I2C_CloseSendData(I2C_Handle_t* pI2C_Handle);

/**
 * @fn I2C_ApplicationEventCallback
 *
 * @brief function for application callback.
 *
 * @param[in] pI2C_Handle handle structure to I2C peripheral.
 * @param[in] app_event application event.
 *
 * @return void.
 */
void I2C_ApplicationEventCallback(I2C_Handle_t* pI2C_Handle, uint8_t app_event);

#endif /* I2C_DRIVER_H */
