/********************************************************************************************************//**
* @file spi_driver.h
*
* @brief Header file containing the prototypes of the APIs for configuring the SPI peripheral.
*
* Public Functions:
*       - void    SPI_Init(SPI_RegDef_t* pSPI_Handle)
*       - void    SPI_DeInit(SPI_RegDef_t* pSPIx)
*       - void    SPI_PerClkCtrl(SPI_RegDef_t* pSPIx, uint8_t en_or_di)
*       - void    SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len)
*       - void    SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t len)
*       - uint8_t SPI_SendDataIT(SPI_Handle_t* pSPI_Handle, uint8_t* pTxBuffer, uint32_t len)
*       - uint8_t SPI_ReceiveDataIT(SPI_Handle_t* pSPI_Handle, uint8_t* pRxBuffer, uint32_t len)
*       - void    SPI_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       - void    SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
*       - void    SPI_IRQHandling(SPI_Handle_t* pSPI_Handle)
*       - void    SPI_Enable(SPI_RegDef_t *pSPIx, uint8_t en_or_di)
*       - void    SPI_SSICfg(SPI_RegDef_t* pSPIx, uint8_t en_or_di)
*       - void    SPI_SSOECfg(SPI_RegDef_t* pSPIx, uint8_t en_or_di)
*       - uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t flagname)
*       - void    SPI_ClearOVRFlag(SPI_RegDef_t* pSPIx)
*       - void    SPI_CloseTx(SPI_Handle_t* pSPI_Handle)
*       - void    SPI_CloseRx(SPI_Handle_t* pSPI_Handle)
*       - void    SPI_ApplicationEventCallback(SPI_Handle_t* pSPI_Handle, uint8_t app_event)
**/

#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include <stdint.h>
#include "stm32f446xx.h"

/**
 * @name SPI possible device modes.
 * @{
 */
#define SPI_DEV_MODE_MASTER     1   /**< @brief Master mode */
#define SPI_DEV_MODE_SLAVE      0   /**< @brief Slave mode */
/**@}*/

/**
 * @name SPI possible bus configuration.
 * @{
 */
#define SPI_BUS_CFG_FD          0   /**< @brief Full Duplex */
#define SPI_BUS_CFG_HD          1   /**< @brief Half Duplex */
#define SPI_BUS_CFG_S_RXONLY    2   /**< @brief Simplex RX Only */
/**@}*/

/**
 * @name SPI possible baud rate control.
 * @{
 */
#define SPI_SCLK_SPEED_DIV2     0   /**< @brief Serial clock speed division by 2 */
#define SPI_SCLK_SPEED_DIV4     1   /**< @brief Serial clock speed division by 4 */
#define SPI_SCLK_SPEED_DIV8     2   /**< @brief Serial clock speed division by 8 */
#define SPI_SCLK_SPEED_DIV16    3   /**< @brief Serial clock speed division by 16 */
#define SPI_SCLK_SPEED_DIV32    4   /**< @brief Serial clock speed division by 32 */
#define SPI_SCLK_SPEED_DIV64    5   /**< @brief Serial clock speed division by 64 */
#define SPI_SCLK_SPEED_DIV128   6   /**< @brief Serial clock speed division by 128 */
#define SPI_SCLK_SPEED_DIV256   7   /**< @brief Serial clock speed division by 256 */
/**@}*/

/**
 * @name SPI possible data frame format.
 * @{
 */
#define SPI_DFF_8BITS   0   /**< @brief 8 bits data frame format */
#define SPI_DFF_16BITS  1   /**< @brief 16 bits data frame format */
/**@}*/

/**
 * @name SPI possible clock polarity.
 * @{
 */
#define SPI_CPOL_HIGH   1   /**< @brief Clock polarity high */
#define SPI_CPOL_LOW    0   /**< @brief Clock polarity low */
/**@}*/

/**
 * @name SPI possible clock phase.
 * @{
 */
#define SPI_CPHA_HIGH   1   /**< @brief Clock phase high */
#define SPI_CPHA_LOW    0   /**< @brief Clock phase low */
/**@}*/

/**
 * @name SPI software slave management configuration.
 * @{
 */
#define SPI_SSM_EN      1   /**< @brief Software slave management enable */
#define SPI_SSM_DI      0   /**< @brief Software slave management disable */
/**@}*/

/**
 * @name SPI related status flags definitions.
 * @{
 */
#define SPI_TXE_FLAG    (1 << SPI_SR_TXE)   /**< @brief TXE of SPI status register */
#define SPI_RXNE_FLAG   (1 << SPI_SR_RXNE)  /**< @brief RXNE of SPI status register */
#define SPI_BUSY_FLAG   (1 << SPI_SR_BSY)   /**< @brief BSY of SPI status register */
/**@}*/

/**
 * @name SPI possible application states
 * @{
 */
#define SPI_READY       0   /**< @brief SPI ready */
#define SPI_BUSY_IN_RX  1   /**< @brief SPI busy in reception */
#define SPI_BUSY_IN_TX  2   /**< @brief SPI busy in transmission */
/**@}*/

/**
 * @name SPI possible application events
 * @{
 */
#define SPI_EVENT_TX_CMPLT  1   /**< @brief Transmission completed event */
#define SPI_EVENT_RX_CMPLT  2   /**< @brief Received completed event */
#define SPI_EVENT_OVR_ERR   3   /**< @brief Overrun error event */
/**@}*/

/**
 * @brief Configuration structure for SPI peripheral.
 */
typedef struct
{
    uint8_t SPI_DeviceMode;     /**< Possible values from @SPI_DEVICE_MODE */
    uint8_t SPI_BusConfig;      /**< Possible values from @SPI_BUS_CFG */
    uint8_t SPI_SclkSpeed;      /**< Possible values from @SPI_SCLK_SPEED */
    uint8_t SPI_DFF;            /**< Possible values from @SPI_DFF */
    uint8_t SPI_CPOL;           /**< Possible values from @SPI_CPOL */
    uint8_t SPI_CPHA;           /**< Possible values from @SPI_CPHA */
    uint8_t SPI_SSM;            /**< Possible values from @SPI_SSM */
}SPI_Config_t;

/**
 * @brief Handle structure for SPIx peripheral.
 */
typedef struct
{
    SPI_RegDef_t* pSPIx;            /**< Base address of the SPIx peripheral */
    SPI_Config_t SPIConfig;         /**< SPIx peripheral configuration settings */
    uint8_t* pTxBuffer;             /**< To store the app. Tx buffer address */
    uint8_t* pRxBuffer;             /**< To store the app. Rx buffer address */
    uint32_t TxLen;                 /**< To store Tx len */
    uint32_t RxLen;                 /**< To store Rx len */
    uint8_t TxState;                /**< To store Tx state */
    uint8_t RxState;                /**< To store Rx state */
}SPI_Handle_t;

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to initialize SPI peripheral.
 * @param[in] pSPI_Handle handle structure for the SPI peripheral.
 * @return void
 */
void SPI_Init(SPI_Handle_t* pSPI_Handle);

/**
 * @brief Function to reset all register of a SPI peripheral.
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @return void
 */
void SPI_DeInit(SPI_RegDef_t* pSPIx);

/**
 * @brief Function to control the peripheral clock of the SPI peripheral.
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @param[in] en_or_di for enable or disable.
 * @return void
 */
void SPI_PerClkCtrl(SPI_RegDef_t* pSPIx, uint8_t en_or_di);

/**
 * @brief Function to send data.
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @param[in] pTxBuffer buffer with the data to send.
 * @param[in] len length of the data to send.
 * @return void
 * @note blocking call.
 */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len);

/**
 * @brief Function to receive data.
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @param[out] pRxBuffer buffer to store the received data.
 * @param[in] len length of the data to receive.
 * @return void
 * @note blocking call.
 */
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t len);

/**
 * @brief Function to send data.
 * @param[in] pSPI_Handle handle structure for the SPI peripheral.
 * @param[in] pTxBuffer buffer with the data to send.
 * @param[in] len length of the data to send.
 * @return @SPI_APP_STATE.
 */
uint8_t SPI_SendDataIT(SPI_Handle_t* pSPI_Handle, uint8_t* pTxBuffer, uint32_t len);

/**
 * @brief Function to receive data.
 * @param[in] pSPI_Handle handle structure for the SPI peripheral.
 * @param[out] pRxBuffer buffer to store the received data.
 * @param[in] len length of the data to receive.
 * @return @SPI_APP_STATE.
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t* pSPI_Handle, uint8_t* pRxBuffer, uint32_t len);

/**
 * @brief Function to configure the IRQ number of the SPI peripheral.
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] en_or_di for enable or disable.
 * @return void.
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di);

/**
 * @brief Function to configure the IRQ number of the SPI peripheral.
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] IRQPriority priority of the interrupt.
 * @return void.
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @brief Function to handle the interrupt of the SPI peripheral.
 * @param[in] pSPI_Handle handle structure to SPI peripheral.
 * @return void.
 */
void SPI_IRQHandling(SPI_Handle_t* pSPI_Handle);

/**
 * @brief Function enable the SPI peripheral.
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @param[in] en_or_di for enable or disable.
 * @return void.
 */
void SPI_Enable(SPI_RegDef_t* pSPIx, uint8_t en_or_di);

/**
 * @brief Function enable the SSI PIN of the SPI.
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @param[in] en_or_di for enable or disable.
 * @return void.
 */
void SPI_SSICfg(SPI_RegDef_t* pSPIx, uint8_t en_or_di);

/**
 * @brief Function enable the SSOE of the SPI.
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @param[in] en_or_di for enable or disable.
 * @return void.
 */
void SPI_SSOECfg(SPI_RegDef_t* pSPIx, uint8_t en_or_di);

/**
 * @brief Function returns the status of a given flag.
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @param[in] flagname the name of the flag.
 * @return flag status: FLAG_SET or FLAG_RESET.
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t flagname);

/**
 * @brief Function clear the OVR flag.
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @return void.
 */
void SPI_ClearOVRFlag(SPI_RegDef_t* pSPIx);

/**
 * @brief Function for closing the SPI transmission.
 * @param[in] pSPI_Handle handle structure to SPI peripheral.
 * @return void.
 */
void SPI_CloseTx(SPI_Handle_t* pSPI_Handle);

/**
 * @brief Function for closing the SPI reception.
 * @param[in] pSPI_Handle handle structure to SPI peripheral.
 * @return void.
 */
void SPI_CloseRx(SPI_Handle_t* pSPI_Handle);

/**
 * @brief Function for application callback.
 * @param[in] pSPI_Handle handle structure to SPI peripheral.
 * @param[in] app_event application event.
 * @return void.
 */
void SPI_ApplicationEventCallback(SPI_Handle_t* pSPI_Handle, uint8_t app_event);

#endif
