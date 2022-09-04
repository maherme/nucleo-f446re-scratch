/********************************************************************************************************//**
* @file dma_driver.c
*
* @brief File containing the APIs for configuring the DMA peripheral.
*
* Public Functions:
*       - void    DMA_Init(DMA_Handle_t* pDMA_Handle)
*       - void    DMA_DeInit(DMA_RegDef_t* pDMAx)
*       - void    DMA_PerClkCtrl(DMA_RegDef_t* pDMAx, uint8_t en_or_di)
*       - void    DMA_Stream_Init(DMA_Stream_Handle_t* pDMA_Stream_Handle)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include "dma_driver.h"

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void DMA_Init(DMA_Handle_t* pDMA_Handle){

    /* Enable the peripheral clock */
    DMA_PerClkCtrl(pDMA_Handle->pDMAx, ENABLE);
}

void DMA_DeInit(DMA_RegDef_t* pDMAx){

    if(pDMAx == DMA1){
        DMA1_REG_RESET();
    }
    else if(pDMAx == DMA2){
        DMA2_REG_RESET();
    }
    else{
        /* do nothing */
    }
}

void DMA_PerClkCtrl(DMA_RegDef_t* pDMAx, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        if(pDMAx == DMA1){
            DMA1_PCLK_EN();
        }
        else if(pDMAx == DMA2){
            DMA2_PCLK_EN();
        }
        else{
            /* do nothing */
        }
    }
    else{
        if(pDMAx == DMA1){
            DMA1_PCLK_DI();
        }
        else if(pDMAx == DMA2){
            DMA2_PCLK_DI();
        }
        else{
            /* do nothing */
        }
    }
}

void DMA_Stream_Init(DMA_Stream_Handle_t* pDMA_Stream_Handle){

    /* Set source address */
    pDMA_Stream_Handle->pStreamx->M0AR = pDMA_Stream_Handle->Stream_Config.src_addr;

    /* Set destination address */
    pDMA_Stream_Handle->pStreamx->PAR = pDMA_Stream_Handle->Stream_Config.dst_addr;

    /* Set number of data items to send */
    pDMA_Stream_Handle->pStreamx->NDTR = (uint32_t)(pDMA_Stream_Handle->Stream_Config.num_data_register);

    /* Set direction of the data transfer */
    pDMA_Stream_Handle->pStreamx->CR &= ~(0x3 << DMA_SCR_DIR);
    pDMA_Stream_Handle->pStreamx->CR |= (pDMA_Stream_Handle->Stream_Config.direction << DMA_SCR_DIR);

    /* Set memory data size */
    pDMA_Stream_Handle->pStreamx->CR &= ~(0x3 << DMA_SCR_MSIZE);
    pDMA_Stream_Handle->pStreamx->CR |= (pDMA_Stream_Handle->Stream_Config.msize << DMA_SCR_MSIZE);

    /* Set peripheral data size */
    if(pDMA_Stream_Handle->Stream_Config.direction != M2M){
        pDMA_Stream_Handle->pStreamx->CR &= ~(0x3 << DMA_SCR_PSIZE);
        pDMA_Stream_Handle->pStreamx->CR |= (pDMA_Stream_Handle->Stream_Config.psize << DMA_SCR_PSIZE);
    }

    /* Set circular mode */
    pDMA_Stream_Handle->pStreamx->CR &= ~(0x1 << DMA_SCR_CIRC);
    pDMA_Stream_Handle->pStreamx->CR |= (pDMA_Stream_Handle->Stream_Config.circular_mode << DMA_SCR_CIRC);

    /* Set priority level */
    pDMA_Stream_Handle->pStreamx->CR &= ~(0x3 << DMA_SCR_PL);
    pDMA_Stream_Handle->pStreamx->CR |= (pDMA_Stream_Handle->Stream_Config.priority << DMA_SCR_PL);

    /* Set channel number */
    pDMA_Stream_Handle->pStreamx->CR &= ~(0x7 << DMA_SCR_CHSEL);
    pDMA_Stream_Handle->pStreamx->CR |= (pDMA_Stream_Handle->Stream_Config.ch_number << DMA_SCR_CHSEL);

    /* Set direct of FIFO mode */
    pDMA_Stream_Handle->pStreamx->FCR &= ~(0x1 << DMA_SFCR_DMDIS);
    pDMA_Stream_Handle->pStreamx->FCR |= (pDMA_Stream_Handle->Stream_Config.mode << DMA_SFCR_DMDIS);

    /* Set FIFO threshold selection */
    if(pDMA_Stream_Handle->Stream_Config.mode == FIFO_MODE){
        pDMA_Stream_Handle->pStreamx->FCR &= ~(0x3 << DMA_SFCR_FTH);
        pDMA_Stream_Handle->pStreamx->FCR |= (pDMA_Stream_Handle->Stream_Config.FIFO_thres << DMA_SFCR_FTH);
    }

    /* Enable de stream */
    pDMA_Stream_Handle->pStreamx->CR |= (1 << DMA_SCR_EN);
}
