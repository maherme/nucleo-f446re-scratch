/********************************************************************************************************//**
* @file dma_driver.h
*
* @brief Header file containing the prototypes of the APIs for configuring the DMA peripheral.
*
* Public Functions:
*       - void      DMA_Init(DMA_Handle_t* pDMA_Handle)
*       - void      DMA_DeInit(DMA_RegDef_t* pDMAx)
*       - void      DMA_PerClkCtrl(DMA_RegDef_t* pDMAx, uint8_t en_or_di)
*       - void      DMA_Stream_Init(DMA_Stream_Handle_t* pDMA_Stream_Handle)
*       - void      DMA_Stream_Enable(DMA_Stream_Handle_t* pDMA_Stream_Handle){
*       - void      DMA_Stream_Set_NDTR(DMA_Stream_Handle_t* pDMA_Stream_Handle, uint32_t ndtr)
*       - void      DMA_Clear_Half_Transfer_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num)
*       - void      DMA_Clear_Transfer_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num)
*       - void      DMA_Clear_Direct_Mode_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num)
*       - void      DMA_Clear_FIFO_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num)
*       - uint32_t  DMA_Get_Transfer_Compl_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num)
*       - uint32_t  DMA_Get_Half_Transfer_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num)
*       - uint32_t  DMA_Get_Transfer_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num)
*       - uint32_t  DMA_Get_Direct_Mode_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num)
*       - uint32_t  DMA_Get_FIFO_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num)
*       - void      DMA_IRQHandling(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num)
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
 * @brief Enum for selecting the stream number of DMA
 */
typedef enum
{
    STREAM0,
    STREAM1,
    STREAM2,
    STREAM3,
    STREAM4,
    STREAM5,
    STREAM6,
    STREAM7
}DMA_Stream_Num_t;

/**
 * @brief Enum for selecting the application event callback of DMA interrupt
 */
typedef enum
{
    TRANSFER_COMPLETE,
    HALF_TRANSFER,
    TRANSFER_ERROR,
    DIRECT_MODE_ERROR,
    FIFO_ERROR
}DMA_App_Event_t;

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
    DMA_Stream_Dsize_t msize;               /**< Memory data size */
    uint8_t minc;                           /**< Memory increment mode, use ENABLE or DISABLE */
    DMA_Stream_Dsize_t psize;               /**< Peripheral data size */
    uint8_t pinc;                           /**< Peripheral increment mode, use ENABLE or DISABLE */
    DMA_Stream_Mode_t mode;                 /**< Direct of FIFO mode option */
    DMA_Stream_FIFO_Thres_t FIFO_thres;     /**< FIFO threshold selection */
    uint8_t circular_mode;                  /**< Use ENABLE or DISABLE */
    DMA_Stream_Priority_t priority;         /**< Priority level */
    uint8_t ch_number;                      /**< Channel number */
    uint8_t HTIE;                           /**< Half Transfer Interrupt Enable */
    uint8_t TCIE;                           /**< Transfer Complete Interrupt Enable */
    uint8_t TEIE;                           /**< Transfer Error Interrupt Enable */
    uint8_t FEIE;                           /**< FIFO overrun/underrun Error Interrupt Enable */
    uint8_t DMEIE;                          /**< Direct Mode Error Interrupt Enable */
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
 * @param[in] pDMA_Handle handle structure for the DMA peripheral.
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

/**
 * @brief Function to initialize Stream of DMA.
 * @param[in] pDMA_Stream_Handle handle structure for the DMA peripheral.
 * @return void
 */
void DMA_Stream_Init(DMA_Stream_Handle_t* pDMA_Stream_Handle);

/**
 * @brief Function to enable the Stream of DMA.
 * @param[in] pDMA_Stream_Handle handle structure for the DMA peripheral.
 * @return void
 */
void DMA_Stream_Enable(DMA_Stream_Handle_t* pDMA_Stream_Handle);

/**
 * @brief Function to set the number of data to be transfered.
 * @param[in] pDMA_Stream_Handle handle structure for the DMA peripheral.
 * @param[in] ndtr is the number of data to be transfered.
 * @return void
 */
void DMA_Stream_Set_NDTR(DMA_Stream_Handle_t* pDMA_Stream_Handle, uint32_t ndtr);

/**
 * @brief Function to clear the transfer complete interrupt flag.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] Stream_Num is the number of stream for clearing flag.
 * @return void
 */
void DMA_Clear_Transfer_Compl_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num);

/**
 * @brief Function to clear the half transfer interrupt flag.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] Stream_Num is the number of stream for clearing flag.
 * @return void
 */
void DMA_Clear_Half_Transfer_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num);

/**
 * @brief Function to clear the transfer error interrupt flag.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] Stream_Num is the number of stream for clearing flag.
 * @return void
 */
void DMA_Clear_Transfer_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num);

/**
 * @brief Function to clear the direct mode error interrupt flag.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] Stream_Num is the number of stream for clearing flag.
 * @return void
 */
void DMA_Clear_Direct_Mode_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num);

/**
 * @brief Function to clear the FIFO error interrupt flag.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] Stream_Num is the number of stream for clearing flag.
 * @return void
 */
void DMA_Clear_FIFO_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num);

/**
 * @brief Function to get the transfer complete interrupt flag.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] Stream_Num is the number of stream for clearing flag.
 * @return 0 if flag is not set
 * @return not 0 if flag is set
 */
uint32_t DMA_Get_Transfer_Compl_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num);

/**
 * @brief Function to get the half transfer interrupt flag.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] Stream_Num is the number of stream for clearing flag.
 * @return 0 if flag is not set
 * @return not 0 if flag is set
 */
uint32_t DMA_Get_Half_Transfer_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num);

/**
 * @brief Function to get the transfer error interrupt flag.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] Stream_Num is the number of stream for clearing flag.
 * @return 0 if flag is not set
 * @return not 0 if flag is set
 */
uint32_t  DMA_Get_Transfer_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num);

/**
 * @brief Function to get the direct mode error interrupt flag.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] Stream_Num is the number of stream for clearing flag.
 * @return 0 if flag is not set
 * @return not 0 if flag is set
 */
uint32_t  DMA_Get_Direct_Mode_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num);

/**
 * @brief Function to get the FIFO error interrupt flag.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] Stream_Num is the number of stream for clearing flag.
 * @return 0 if flag is not set
 * @return not 0 if flag is set
 */
uint32_t  DMA_Get_FIFO_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num);

/**
 * @brief Function for handling the DMA interrupts.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] Stream_Num is the number of stream for clearing flag.
 * @return void
 */
void DMA_IRQHandling(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num);

#endif /* DMA_DRIVER_H */
