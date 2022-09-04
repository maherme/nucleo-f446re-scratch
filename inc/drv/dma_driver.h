/********************************************************************************************************//**
* @file dma_driver.h
*
* @brief Header file containing the prototypes of the APIs for configuring the DMA peripheral.
*
* Public Functions:
*       - void    DMA_Init(DMA_Handle_t* pDMA_Handle)
*       - void    DMA_DeInit(DMA_RegDef_t* pDMAx)
*       - void    DMA_PerClkCtrl(DMA_RegDef_t* pDMAx, uint8_t en_or_di)
*/

#ifndef DMA_DRIVER_H
#define DMA_DRIVER_H

#include "stm32f446xx.h"

/**
 * @brief Enum for selecting the direction of the stream.
 */
typedef enum
{
    P2M,    /**< Peripheral to memory direction */
    M2P,    /**< Memory to peripheral direction */
    M2M     /**< Memory to memory direction */
}DMA_Stream_Dir_t;

/**
 * @brief Enum for selecting the memory/peripheral data size
 */
typedef enum
{
    BYTE,       /**< Byte (8-bits) */
    HWORD,      /**< Half-word (16-bits) */
    WORD        /**< Word (32-bits) */
}DMA_Stream_Dsize_t;

/**
 * @brief Enum for selecting the direct of FIFO mode
 */
typedef enum
{
    DIRECT_MODE,    /**< Direct mode enable */
    FIFO_MODE       /**< Direct mode disable, so FIFO is enable */
}DMA_Stream_Mode_t;

/**
 * @brief Enum for selecting the FIFO threshold selection
 */
typedef enum
{
    QUARTER,        /**< 1/4 full FIFO */
    MIDDLE,         /**< 1/2 full FIFO */
    THREE_QUARTER,  /**< 3/4 full FIFO */
    FULL            /**< full FIFO */
}DMA_Stream_FIFO_Thres_t;

/**
 * @brief Enum for selecting the priority level
 */
typedef enum
{
    LOW,        /**< Low level */
    MEDIUM,     /**< Medium level */
    HIGH,       /**< High level */
    VERY_HIGH   /**< Maximum level */
}DMA_Stream_Priority_t;

/**
 * @brief Configuration structure for DMA peripheral.
 */
typedef struct
{
}DMA_Config_t;

/**
 * @brief Configuration structure for DMA stream.
 */
typedef struct
{
    uint32_t src_addr;                      /**< Source address */
    uint32_t dst_addr;                      /**< Destination address */
    uint16_t num_data_register;             /**< Number of data items to transfer */
    DMA_Stream_Dir_t direction;             /**< Data transfer direction */
    DMA_Stream_Dsize_t msize;               /** < Memory data size */
    DMA_Stream_Dsize_t psize;               /**< Peripheral data size */
    DMA_Stream_Mode_t mode;                 /**< Direct of FIFO mode option */
    DMA_Stream_FIFO_Thres_t FIFO_thres;     /**< FIFO threshold selection */
    uint8_t circular_mode;                  /**< Use ENABLE or DISABLE */
    DMA_Stream_Priority_t priority;         /**< Priority level */
    uint8_t ch_number;                      /**< Channel number */
}DMA_Stream_Config_t;

/**
 * @brief Handle structure for DMA peripheral.
 */
typedef struct
{
    DMA_RegDef_t* pDMAx;            /**< Base address of the DMAx peripheral */
    DMA_Config_t DMA_Config;        /**< DMAx peripheral configuration settings */
}DMA_Handle_t;

/**
 * @brief Handle structure for DMA stream.
 */
typedef struct
{
    DMA_Stream_RegDef_t* pStreamx;      /**< Base address of the DMA stream x */
    DMA_Stream_Config_t Stream_Config;  /**< Stream x configuration settings */
}DMA_Stream_Handle_t;

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to initialize DMA peripheral.
 * @param[in] pDMA handle structure for the DMA peripheral.
 * @return void
 */
void DMA_Init(DMA_Handle_t* pDMA_Handle);

/**
 * @brief Function to reset all register of a DMA peripheral.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @return void
 */
void DMA_DeInit(DMA_RegDef_t* pDMAx);

/**
 * @brief Function to control the peripheral clock of the DMA peripheral.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] en_or_di for enable or disable.
 * @return void
 */
void DMA_PerClkCtrl(DMA_RegDef_t* pDMAx, uint8_t en_or_di);

#endif /* DMA_DRIVER_H */
