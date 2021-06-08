/*****************************************************************************************************
* FILENAME :        spi_driver.h
*
* DESCRIPTION :
*       Header file containing the prototypes of the APIs for configuring the SPI peripheral.
*
* PUBLIC FUNCTIONS :
*       void    SPI_Init(SPI_RegDef_t* pSPI_Handle)
*       void    SPI_DeInit(SPI_RegDef_t* pSPIx)
*       void    SPI_PerClkCtrl(SPI_RegDef_t* pSPIx, uint8_t en_or_di)
*       void    SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len)
*       void    SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t len)
*       uint8_t SPI_SendDataIT(SPI_Handle_t* pSPI_Handle, uint8_t* pTxBuffer, uint32_t len)
*       void    SPI_ReceiveDataIT(SPI_Handle_t* pSPI_Handle, uint8_t* pRxBuffer, uint32_t len)
*       void    SPI_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       void    SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
*       void    SPI_IRQHandling(SPI_Handle_t* pHandle)
*       void    SPI_Enable(SPI_RegDef_t *pSPIx, uint8_t en_or_di)
*       void    SPI_SSICfg(SPI_RegDef_t* pSPIx, uint8_t en_or_di)
*
**/

#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include <stdint.h>
#include "stm32f446xx.h"

/**
 * @SPI_DEVICE_MODE
 * SPI possible device modes.
 */
#define SPI_DEV_MODE_MASTER     1
#define SPI_DEV_MODE_SLAVE      0

/**
 * @SPI_BUS_CFG
 * SPI possible bus configuration.
 */
#define SPI_BUS_CFG_FD          0   /* Full Duplex */
#define SPI_BUS_CFG_HD          1   /* Half Duplex */
#define SPI_BUS_CFG_S_RXONLY    2   /* Simplex RX Only */

/**
 * @SPI_SCLK_SPEED
 * SPI possible baud rate control.
 */
#define SPI_SCLK_SPEED_DIV2     0
#define SPI_SCLK_SPEED_DIV4     1
#define SPI_SCLK_SPEED_DIV8     2
#define SPI_SCLK_SPEED_DIV16    3
#define SPI_SCLK_SPEED_DIV32    4
#define SPI_SCLK_SPEED_DIV64    5
#define SPI_SCLK_SPEED_DIV128   6
#define SPI_SCLK_SPEED_DIV256   7

/**
 * @SPI_DFF
 * SPI possible data frame format.
 */
#define SPI_DFF_8BITS   0
#define SPI_DFF_16BITS  1

/**
 * @SPI_CPOL
 * SPI possible clock polarity.
 */
#define SPI_CPOL_HIGH   1
#define SPI_CPOL_LOW    0

/**
 * @SPI_CPHA
 * SPI possible clock phase.
 */
#define SPI_CPHA_HIGH   1
#define SPI_CPHA_LOW    0

/**
 * @SPI_SSM
 * SPI software slave management configuration.
 */
#define SPI_SSM_EN      1
#define SPI_SSM_DI      0

/**
 * SPI related status flags definitions.
 */
#define SPI_TXE_FLAG    (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   (1 << SPI_SR_BSY)

/**
 * @SPI_APP_STATE
 * SPI possible application states
 */
#define SPI_READY       0
#define SPI_BUSY_IN_RX  1
#define SPI_BUSY_IN_TX  2

/**
 * Configuration structure for SPI peripheral.
 */
typedef struct
{
    uint8_t SPI_DeviceMode;     /* Possible values from @SPI_DEVICE_MODE */
    uint8_t SPI_BusConfig;      /* Possible values from @SPI_BUS_CFG */
    uint8_t SPI_SclkSpeed;      /* Possible values from @SPI_SCLK_SPEED */
    uint8_t SPI_DFF;            /* Possible values from @SPI_DFF */
    uint8_t SPI_CPOL;           /* Possible values from @SPI_CPOL */
    uint8_t SPI_CPHA;           /* Possible values from @SPI_CPHA */
    uint8_t SPI_SSM;            /* Possible values from @SPI_SSM */
}SPI_Config_t;

/**
 * Handle structure for SPIx peripheral.
 */
typedef struct
{
    SPI_RegDef_t* pSPIx;            /* Base address of the SPIx peripheral */
    SPI_Config_t SPIConfig;         /* SPIx peripheral configuration settings */
    uint8_t* pTxBuffer;             /* To store the app. Tx buffer address */
    uint8_t* pRxBuffer;             /* To store the app. Rx buffer address */
    uint32_t TxLen;                 /* To store Tx len */
    uint32_t RxLen;                 /* To store Rx len */
    uint8_t TxState;                /* To store Tx state */
    uint8_t RxState;                /* To store Rx state */
}SPI_Handle_t;

/*****************************************************************************************************/
/*                                       APIs supported                                              */
/*****************************************************************************************************/

/**
 * @fn SPI_Init
 *
 * @brief function to initialize SPI peripheral.
 *
 * @param[in] pSPI_Handle handle structure for the SPI peripheral.
 *
 * @return void
 */
void SPI_Init(SPI_Handle_t* pSPI_Handle);

/**
 *@fn SPI_DeInit
 *
 * @brief function to reset all register of a SPI peripheral.
 *
 * @param[in] pSPIx the base address of the SPIx peripheral.
 *
 * @return void
 */
void SPI_DeInit(SPI_RegDef_t* pSPIx);

/**
 * @fn SPI_PerClkCtrl
 *
 * @brief function to control the peripheral clock of the SPI peripheral.
 *
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @param[in] en_or_di for enable or disable.
 *
 * @return void
 */
void SPI_PerClkCtrl(SPI_RegDef_t* pSPIx, uint8_t en_or_di);

/**
 * @fn SPI_SendData
 *
 * @brief function to send data.
 *
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @param[in] pTxBuffer buffer with the data to send.
 * @param[in] len length of the data to send.
 *
 * @return void
 *
 * @note blocking call.
 */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len);

/**
 * @fn SPI_ReceiveData
 *
 * @brief function to receive data.
 *
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @param[in] pRxBuffer buffer to store the received data.
 * @param[in] len length of the data to receive.
 *
 * @return void
 *
 * @note blocking call.
 */
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t len);

/**
 * @fn SPI_SendDataIT
 *
 * @brief function to send data.
 *
 * @param[in] pSPI_Handle handle structure for the SPI peripheral.
 * @param[in] pTxBuffer buffer with the data to send.
 * @param[in] len length of the data to send.
 *
 * @return @SPI_APP_STATE.
 *
 */
uint8_t SPI_SendDataIT(SPI_Handle_t* pSPI_Handle, uint8_t* pTxBuffer, uint32_t len);

/**
 * @fn SPI_ReceiveDataIT
 *
 * @brief function to receive data.
 *
 * @param[in] pSPI_Handle handle structure for the SPI peripheral.
 * @param[in] pRxBuffer buffer to store the received data.
 * @param[in] len length of the data to receive.
 *
 * @return @SPI_APP_STATE.
 *
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t* pSPI_Handle, uint8_t* pRxBuffer, uint32_t len);

/**
 * @fn SPI_IRQConfig
 *
 * @brief function to configure the IRQ number of the SPI peripheral.
 *
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] en_or_di for enable or disable.
 *
 * @return void.
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di);

/**
 * @fn SPI_IRQConfig
 *
 * @brief function to configure the IRQ number of the SPI peripheral.
 *
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] IRQPriority priority of the interrupt.
 *
 * @return void.
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @fn SPI_IRQHandling
 *
 * @brief function to handle the interrupt of the SPI peripheral.
 *
 * @param[in] pHandle handle structure to SPI peripheral.
 *
 * @return void.
 */
void SPI_IRQHandling(SPI_Handle_t* pHandle);

/**
 * @fn SPI_Enable
 *
 * @brief function enable the SPI peripheral.
 *
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @param[in] en_or_di for enable or disable.
 *
 * @return void.
 */
void SPI_Enable(SPI_RegDef_t* pSPIx, uint8_t en_or_di);

/**
 * @fn SPI_Enable
 *
 * @brief function enable the SSI PIN of the SPI.
 *
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @param[in] en_or_di for enable or disable.
 *
 * @return void.
 */
void SPI_SSICfg(SPI_RegDef_t* pSPIx, uint8_t en_or_di);

/**
 * @fn SPI_GetFlagStatus
 *
 * @brief function returns the status of a given flag.
 *
 * @param[in] pSPIx the base address of the SPIx peripheral.
 * @param[in] flagname the name of the flag.
 *
 * @return flag status: FLAG_SET or FLAG_RESET.
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t flagname);

#endif 
