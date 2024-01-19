/********************************************************************************************************//**
* @file dma_driver.c
*
* @brief File containing the APIs for configuring the DMA peripheral.
*
* Public Functions:
*       - void      DMA_Init(DMA_Handle_t* pDMA_Handle)
*       - void      DMA_DeInit(DMA_RegDef_t* pDMAx)
*       - void      DMA_PerClkCtrl(DMA_RegDef_t* pDMAx, uint8_t en_or_di)
*       - void      DMA_Stream_Init(DMA_Stream_Handle_t* pDMA_Stream_Handle)
*       - void      DMA_Stream_Enable(DMA_Stream_Handle_t* pDMA_Stream_Handle){
*       - void      DMA_Stream_Set_NDTR(DMA_Stream_Handle_t* pDMA_Stream_Handle, uint32_t ndtr)
*       - void      DMA_Clear_Transfer_Compl_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num)
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
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include "dma_driver.h"

/***********************************************************************************************************/
/*                                       Static Function Prototypes                                        */
/***********************************************************************************************************/

/**
 * @brief Function to manage the transfer complete interrupt.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] Stream_Num is the number of stream for clearing flag.
 * @return void
 */
static void Transfer_Complete_Callback(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num);

/**
 * @brief Function to manage the half transfer interrupt.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] Stream_Num is the number of stream for clearing flag.
 * @return void
 */
static void Half_Transfer_Callback(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num);

/**
 * @brief Function to manage the transfer error interrupt.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] Stream_Num is the number of stream for clearing flag.
 * @return void
 */
static void Transfer_Error_Callback(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num);

/**
 * @brief Function to manage the direct mode error interrupt.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] Stream_Num is the number of stream for clearing flag.
 * @return void
 */
static void Direct_Mode_Error_Callback(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num);

/**
 * @brief Function to manage the FIFO error interrupt.
 * @param[in] pDMAx the base address of the DMAx peripheral.
 * @param[in] Stream_Num is the number of stream for clearing flag.
 * @return void
 */
static void FIFO_Error_Callback(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num);

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

    /* Set memory increment mode */
    pDMA_Stream_Handle->pStreamx->CR &= ~(0x1 << DMA_SCR_MINC);
    pDMA_Stream_Handle->pStreamx->CR |= (pDMA_Stream_Handle->Stream_Config.minc << DMA_SCR_MINC);

    /* Set peripheral data size */
    if(pDMA_Stream_Handle->Stream_Config.direction != M2M){
        pDMA_Stream_Handle->pStreamx->CR &= ~(0x3 << DMA_SCR_PSIZE);
        pDMA_Stream_Handle->pStreamx->CR |= (pDMA_Stream_Handle->Stream_Config.psize << DMA_SCR_PSIZE);
    }

    /* Set peripheral increment mode */
    if(pDMA_Stream_Handle->Stream_Config.direction != M2M){
        pDMA_Stream_Handle->pStreamx->CR &= ~(0x1 << DMA_SCR_PINC);
        pDMA_Stream_Handle->pStreamx->CR |= (pDMA_Stream_Handle->Stream_Config.pinc << DMA_SCR_PINC);
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

    /* Set Half Transfer IE */
    if(pDMA_Stream_Handle->Stream_Config.HTIE == ENABLE){
        pDMA_Stream_Handle->pStreamx->CR |= (1 << DMA_SCR_HTIE);
    }

    /* Set Transfer Complete IE */
    if(pDMA_Stream_Handle->Stream_Config.TCIE == ENABLE){
        pDMA_Stream_Handle->pStreamx->CR |= (1 << DMA_SCR_TCIE);
    }

    /* Set Transfer Error IE */
    if(pDMA_Stream_Handle->Stream_Config.TEIE == ENABLE){
        pDMA_Stream_Handle->pStreamx->CR |= (1 << DMA_SCR_TEIE);
    }

    /* Set FIFO Error IE */
    if(pDMA_Stream_Handle->Stream_Config.FEIE == ENABLE){
        pDMA_Stream_Handle->pStreamx->FCR |= (1 << DMA_SFCR_FEIE);
    }

    /* Set Direct Mode Error IE */
    if(pDMA_Stream_Handle->Stream_Config.DMEIE == ENABLE){
        pDMA_Stream_Handle->pStreamx->CR |= (1 << DMA_SCR_DMEIE);
    }
}

void DMA_Stream_Enable(DMA_Stream_Handle_t* pDMA_Stream_Handle){

    /* Enable stream */
    pDMA_Stream_Handle->pStreamx->CR |= (1 << DMA_SCR_EN);
}

void DMA_Stream_Set_NDTR(DMA_Stream_Handle_t* pDMA_Stream_Handle, uint32_t ndtr){

    /* Set NDTR value */
    pDMA_Stream_Handle->pStreamx->NDTR = ndtr;
}

void DMA_Clear_Transfer_Compl_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num){

    switch(Stream_Num){
    case STREAM0:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CTCIF0);
        break;
    case STREAM1:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CTCIF1);
        break;
    case STREAM2:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CTCIF2);
        break;
    case STREAM3:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CTCIF3);
        break;
    case STREAM4:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CTCIF4);
        break;
    case STREAM5:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CTCIF5);
        break;
    case STREAM6:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CTCIF6);
        break;
    case STREAM7:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CTCIF7);
        break;
    default:
        break;
    }
}

void DMA_Clear_Half_Transfer_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num){

    switch(Stream_Num){
    case STREAM0:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CHTIF0);
        break;
    case STREAM1:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CHTIF1);
        break;
    case STREAM2:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CHTIF2);
        break;
    case STREAM3:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CHTIF3);
        break;
    case STREAM4:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CHTIF4);
        break;
    case STREAM5:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CHTIF5);
        break;
    case STREAM6:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CHTIF6);
        break;
    case STREAM7:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CHTIF7);
        break;
    default:
        break;
    }
}

void DMA_Clear_Transfer_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num){

    switch(Stream_Num){
    case STREAM0:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CTEIF0);
        break;
    case STREAM1:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CTEIF1);
        break;
    case STREAM2:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CTEIF2);
        break;
    case STREAM3:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CTEIF3);
        break;
    case STREAM4:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CTEIF4);
        break;
    case STREAM5:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CTEIF5);
        break;
    case STREAM6:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CTEIF6);
        break;
    case STREAM7:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CTEIF7);
        break;
    default:
        break;
    }
}

void DMA_Clear_Direct_Mode_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num){

    switch(Stream_Num){
    case STREAM0:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CDMEIF0);
        break;
    case STREAM1:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CDMEIF1);
        break;
    case STREAM2:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CDMEIF2);
        break;
    case STREAM3:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CDMEIF3);
        break;
    case STREAM4:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CDMEIF4);
        break;
    case STREAM5:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CDMEIF5);
        break;
    case STREAM6:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CDMEIF6);
        break;
    case STREAM7:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CDMEIF7);
        break;
    default:
        break;
    }
}

void DMA_Clear_FIFO_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num){

    switch(Stream_Num){
    case STREAM0:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CFEIF0);
        break;
    case STREAM1:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CFEIF1);
        break;
    case STREAM2:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CFEIF2);
        break;
    case STREAM3:
        pDMAx->LIFCR |= (1 << DMA_LIFCR_CFEIF3);
        break;
    case STREAM4:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CFEIF4);
        break;
    case STREAM5:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CFEIF5);
        break;
    case STREAM6:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CFEIF6);
        break;
    case STREAM7:
        pDMAx->HIFCR |= (1 << DMA_HIFCR_CFEIF7);
        break;
    default:
        break;
    }
}

uint32_t DMA_Get_Transfer_Compl_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num){

    uint32_t ret = 0;

    switch(Stream_Num){
    case STREAM0:
        ret = pDMAx->LISR & (1 << DMA_LISR_TCIF0);
        break;
    case STREAM1:
        ret = pDMAx->LISR & (1 << DMA_LISR_TCIF1);
        break;
    case STREAM2:
        ret = pDMAx->LISR & (1 << DMA_LISR_TCIF2);
        break;
    case STREAM3:
        ret = pDMAx->LISR & (1 << DMA_LISR_TCIF3);
        break;
    case STREAM4:
        ret = pDMAx->HISR & (1 << DMA_HISR_TCIF4);
        break;
    case STREAM5:
        ret = pDMAx->HISR & (1 << DMA_HISR_TCIF5);
        break;
    case STREAM6:
        ret = pDMAx->HISR & (1 << DMA_HISR_TCIF6);
        break;
    case STREAM7:
        ret = pDMAx->HISR & (1 << DMA_HISR_TCIF7);
        break;
    default:
        break;
    }

    return ret;
}

uint32_t DMA_Get_Half_Transfer_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num){

    uint32_t ret = 0;

    switch(Stream_Num){
    case STREAM0:
        ret = pDMAx->LISR & (1 << DMA_LISR_HTIF0);
        break;
    case STREAM1:
        ret = pDMAx->LISR & (1 << DMA_LISR_HTIF1);
        break;
    case STREAM2:
        ret = pDMAx->LISR & (1 << DMA_LISR_HTIF2);
        break;
    case STREAM3:
        ret = pDMAx->LISR & (1 << DMA_LISR_HTIF3);
        break;
    case STREAM4:
        ret = pDMAx->HISR & (1 << DMA_HISR_HTIF4);
        break;
    case STREAM5:
        ret = pDMAx->HISR & (1 << DMA_HISR_HTIF5);
        break;
    case STREAM6:
        ret = pDMAx->HISR & (1 << DMA_HISR_HTIF6);
        break;
    case STREAM7:
        ret = pDMAx->HISR & (1 << DMA_HISR_HTIF7);
        break;
    default:
        break;
    }

    return ret;
}

uint32_t DMA_Get_Transfer_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num){

    uint32_t ret = 0;

    switch(Stream_Num){
    case STREAM0:
        ret = pDMAx->LISR & (1 << DMA_LISR_TEIF0);
        break;
    case STREAM1:
        ret = pDMAx->LISR & (1 << DMA_LISR_TEIF1);
        break;
    case STREAM2:
        ret = pDMAx->LISR & (1 << DMA_LISR_TEIF2);
        break;
    case STREAM3:
        ret = pDMAx->LISR & (1 << DMA_LISR_TEIF3);
        break;
    case STREAM4:
        ret = pDMAx->HISR & (1 << DMA_HISR_TEIF4);
        break;
    case STREAM5:
        ret = pDMAx->HISR & (1 << DMA_HISR_TEIF5);
        break;
    case STREAM6:
        ret = pDMAx->HISR & (1 << DMA_HISR_TEIF6);
        break;
    case STREAM7:
        ret = pDMAx->HISR & (1 << DMA_HISR_TEIF7);
        break;
    default:
        break;
    }

    return ret;
}

uint32_t DMA_Get_Direct_Mode_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num){

    uint32_t ret = 0;

    switch(Stream_Num){
    case STREAM0:
        ret = pDMAx->LISR & (1 << DMA_LISR_DMEIF0);
        break;
    case STREAM1:
        ret = pDMAx->LISR & (1 << DMA_LISR_DMEIF1);
        break;
    case STREAM2:
        ret = pDMAx->LISR & (1 << DMA_LISR_DMEIF2);
        break;
    case STREAM3:
        ret = pDMAx->LISR & (1 << DMA_LISR_DMEIF3);
        break;
    case STREAM4:
        ret = pDMAx->HISR & (1 << DMA_HISR_DMEIF4);
        break;
    case STREAM5:
        ret = pDMAx->HISR & (1 << DMA_HISR_DMEIF5);
        break;
    case STREAM6:
        ret = pDMAx->HISR & (1 << DMA_HISR_DMEIF6);
        break;
    case STREAM7:
        ret = pDMAx->HISR & (1 << DMA_HISR_DMEIF7);
        break;
    default:
        break;
    }

    return ret;
}

uint32_t DMA_Get_FIFO_Error_Int_Flag(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num){

    uint32_t ret = 0;

    switch(Stream_Num){
    case STREAM0:
        ret = pDMAx->LISR & (1 << DMA_LISR_FEIF0);
        break;
    case STREAM1:
        ret = pDMAx->LISR & (1 << DMA_LISR_FEIF1);
        break;
    case STREAM2:
        ret = pDMAx->LISR & (1 << DMA_LISR_FEIF2);
        break;
    case STREAM3:
        ret = pDMAx->LISR & (1 << DMA_LISR_FEIF3);
        break;
    case STREAM4:
        ret = pDMAx->HISR & (1 << DMA_HISR_FEIF4);
        break;
    case STREAM5:
        ret = pDMAx->HISR & (1 << DMA_HISR_FEIF5);
        break;
    case STREAM6:
        ret = pDMAx->HISR & (1 << DMA_HISR_FEIF6);
        break;
    case STREAM7:
        ret = pDMAx->HISR & (1 << DMA_HISR_FEIF7);
        break;
    default:
        break;
    }

    return ret;
}

void DMA_IRQHandling(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num){

    if(DMA_Get_Transfer_Compl_Int_Flag(pDMAx, Stream_Num)){
        DMA_Clear_Transfer_Compl_Int_Flag(pDMAx, Stream_Num);
        Transfer_Complete_Callback(pDMAx, Stream_Num);
    }
    else if(DMA_Get_Half_Transfer_Int_Flag(pDMAx, Stream_Num)){
        DMA_Clear_Half_Transfer_Int_Flag(pDMAx, Stream_Num);
        Half_Transfer_Callback(pDMAx, Stream_Num);
    }
    else if(DMA_Get_Transfer_Error_Int_Flag(pDMAx, Stream_Num)){
        DMA_Clear_Transfer_Error_Int_Flag(pDMAx, Stream_Num);
        Transfer_Error_Callback(pDMAx, Stream_Num);
    }
    else if(DMA_Get_Direct_Mode_Error_Int_Flag(pDMAx, Stream_Num)){
        DMA_Clear_Direct_Mode_Error_Int_Flag(pDMAx, Stream_Num);
        Direct_Mode_Error_Callback(pDMAx, Stream_Num);
    }
    else if(DMA_Get_FIFO_Error_Int_Flag(pDMAx, Stream_Num)){
        DMA_Clear_FIFO_Error_Int_Flag(pDMAx, Stream_Num);
        FIFO_Error_Callback(pDMAx, Stream_Num);
    }
    else{
    }
}

/***********************************************************************************************************/
/*                                       Weak Functions                                                    */
/*            This is a weak implementation. The application may override this function                    */
/***********************************************************************************************************/

__attribute__((weak)) void DMA1_Stream0_AppEventCallback(DMA_App_Event_t app_event){ (void)app_event; }
__attribute__((weak)) void DMA1_Stream1_AppEventCallback(DMA_App_Event_t app_event){ (void)app_event; }
__attribute__((weak)) void DMA1_Stream2_AppEventCallback(DMA_App_Event_t app_event){ (void)app_event; }
__attribute__((weak)) void DMA1_Stream3_AppEventCallback(DMA_App_Event_t app_event){ (void)app_event; }
__attribute__((weak)) void DMA1_Stream4_AppEventCallback(DMA_App_Event_t app_event){ (void)app_event; }
__attribute__((weak)) void DMA1_Stream5_AppEventCallback(DMA_App_Event_t app_event){ (void)app_event; }
__attribute__((weak)) void DMA1_Stream6_AppEventCallback(DMA_App_Event_t app_event){ (void)app_event; }
__attribute__((weak)) void DMA1_Stream7_AppEventCallback(DMA_App_Event_t app_event){ (void)app_event; }
__attribute__((weak)) void DMA2_Stream0_AppEventCallback(DMA_App_Event_t app_event){ (void)app_event; }
__attribute__((weak)) void DMA2_Stream1_AppEventCallback(DMA_App_Event_t app_event){ (void)app_event; }
__attribute__((weak)) void DMA2_Stream2_AppEventCallback(DMA_App_Event_t app_event){ (void)app_event; }
__attribute__((weak)) void DMA2_Stream3_AppEventCallback(DMA_App_Event_t app_event){ (void)app_event; }
__attribute__((weak)) void DMA2_Stream4_AppEventCallback(DMA_App_Event_t app_event){ (void)app_event; }
__attribute__((weak)) void DMA2_Stream5_AppEventCallback(DMA_App_Event_t app_event){ (void)app_event; }
__attribute__((weak)) void DMA2_Stream6_AppEventCallback(DMA_App_Event_t app_event){ (void)app_event; }
__attribute__((weak)) void DMA2_Stream7_AppEventCallback(DMA_App_Event_t app_event){ (void)app_event; }

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/

void Transfer_Complete_Callback(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num){

    if(pDMAx == DMA1){
        switch(Stream_Num){
        case STREAM0:
            DMA1_Stream0_AppEventCallback(TRANSFER_COMPLETE);
            break;
        case STREAM1:
            DMA1_Stream1_AppEventCallback(TRANSFER_COMPLETE);
            break;
        case STREAM2:
            DMA1_Stream2_AppEventCallback(TRANSFER_COMPLETE);
            break;
        case STREAM3:
            DMA1_Stream3_AppEventCallback(TRANSFER_COMPLETE);
            break;
        case STREAM4:
            DMA1_Stream4_AppEventCallback(TRANSFER_COMPLETE);
            break;
        case STREAM5:
            DMA1_Stream5_AppEventCallback(TRANSFER_COMPLETE);
            break;
        case STREAM6:
            DMA1_Stream6_AppEventCallback(TRANSFER_COMPLETE);
            break;
        case STREAM7:
            DMA1_Stream7_AppEventCallback(TRANSFER_COMPLETE);
            break;
        default:
            break;
        }
    }
    else if(pDMAx == DMA2){
        switch(Stream_Num){
        case STREAM0:
            DMA2_Stream0_AppEventCallback(TRANSFER_COMPLETE);
            break;
        case STREAM1:
            DMA2_Stream1_AppEventCallback(TRANSFER_COMPLETE);
            break;
        case STREAM2:
            DMA2_Stream2_AppEventCallback(TRANSFER_COMPLETE);
            break;
        case STREAM3:
            DMA2_Stream3_AppEventCallback(TRANSFER_COMPLETE);
            break;
        case STREAM4:
            DMA2_Stream4_AppEventCallback(TRANSFER_COMPLETE);
            break;
        case STREAM5:
            DMA2_Stream5_AppEventCallback(TRANSFER_COMPLETE);
            break;
        case STREAM6:
            DMA2_Stream6_AppEventCallback(TRANSFER_COMPLETE);
            break;
        case STREAM7:
            DMA2_Stream7_AppEventCallback(TRANSFER_COMPLETE);
            break;
        default:
            break;
        }
    }
    else{
        /* do nothing */
    }
}

void Half_Transfer_Callback(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num){

    if(pDMAx == DMA1){
        switch(Stream_Num){
        case STREAM0:
            DMA1_Stream0_AppEventCallback(HALF_TRANSFER);
            break;
        case STREAM1:
            DMA1_Stream1_AppEventCallback(HALF_TRANSFER);
            break;
        case STREAM2:
            DMA1_Stream2_AppEventCallback(HALF_TRANSFER);
            break;
        case STREAM3:
            DMA1_Stream3_AppEventCallback(HALF_TRANSFER);
            break;
        case STREAM4:
            DMA1_Stream4_AppEventCallback(HALF_TRANSFER);
            break;
        case STREAM5:
            DMA1_Stream5_AppEventCallback(HALF_TRANSFER);
            break;
        case STREAM6:
            DMA1_Stream6_AppEventCallback(HALF_TRANSFER);
            break;
        case STREAM7:
            DMA1_Stream7_AppEventCallback(HALF_TRANSFER);
            break;
        default:
            break;
        }
    }
    else if(pDMAx == DMA2){
        switch(Stream_Num){
        case STREAM0:
            DMA2_Stream0_AppEventCallback(HALF_TRANSFER);
            break;
        case STREAM1:
            DMA2_Stream1_AppEventCallback(HALF_TRANSFER);
            break;
        case STREAM2:
            DMA2_Stream2_AppEventCallback(HALF_TRANSFER);
            break;
        case STREAM3:
            DMA2_Stream3_AppEventCallback(HALF_TRANSFER);
            break;
        case STREAM4:
            DMA2_Stream4_AppEventCallback(HALF_TRANSFER);
            break;
        case STREAM5:
            DMA2_Stream5_AppEventCallback(HALF_TRANSFER);
            break;
        case STREAM6:
            DMA2_Stream6_AppEventCallback(HALF_TRANSFER);
            break;
        case STREAM7:
            DMA2_Stream7_AppEventCallback(HALF_TRANSFER);
            break;
        default:
            break;
        }
    }
    else{
        /* do nothing */
    }
}

void Transfer_Error_Callback(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num){

    if(pDMAx == DMA1){
        switch(Stream_Num){
        case STREAM0:
            DMA1_Stream0_AppEventCallback(TRANSFER_ERROR);
            break;
        case STREAM1:
            DMA1_Stream1_AppEventCallback(TRANSFER_ERROR);
            break;
        case STREAM2:
            DMA1_Stream2_AppEventCallback(TRANSFER_ERROR);
            break;
        case STREAM3:
            DMA1_Stream3_AppEventCallback(TRANSFER_ERROR);
            break;
        case STREAM4:
            DMA1_Stream4_AppEventCallback(TRANSFER_ERROR);
            break;
        case STREAM5:
            DMA1_Stream5_AppEventCallback(TRANSFER_ERROR);
            break;
        case STREAM6:
            DMA1_Stream6_AppEventCallback(TRANSFER_ERROR);
            break;
        case STREAM7:
            DMA1_Stream7_AppEventCallback(TRANSFER_ERROR);
            break;
        default:
            break;
        }
    }
    else if(pDMAx == DMA2){
        switch(Stream_Num){
        case STREAM0:
            DMA2_Stream0_AppEventCallback(TRANSFER_ERROR);
            break;
        case STREAM1:
            DMA2_Stream1_AppEventCallback(TRANSFER_ERROR);
            break;
        case STREAM2:
            DMA2_Stream2_AppEventCallback(TRANSFER_ERROR);
            break;
        case STREAM3:
            DMA2_Stream3_AppEventCallback(TRANSFER_ERROR);
            break;
        case STREAM4:
            DMA2_Stream4_AppEventCallback(TRANSFER_ERROR);
            break;
        case STREAM5:
            DMA2_Stream5_AppEventCallback(TRANSFER_ERROR);
            break;
        case STREAM6:
            DMA2_Stream6_AppEventCallback(TRANSFER_ERROR);
            break;
        case STREAM7:
            DMA2_Stream7_AppEventCallback(TRANSFER_ERROR);
            break;
        default:
            break;
        }
    }
    else{
        /* do nothing */
    }
}

void Direct_Mode_Error_Callback(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num){

    if(pDMAx == DMA1){
        switch(Stream_Num){
        case STREAM0:
            DMA1_Stream0_AppEventCallback(DIRECT_MODE_ERROR);
            break;
        case STREAM1:
            DMA1_Stream1_AppEventCallback(DIRECT_MODE_ERROR);
            break;
        case STREAM2:
            DMA1_Stream2_AppEventCallback(DIRECT_MODE_ERROR);
            break;
        case STREAM3:
            DMA1_Stream3_AppEventCallback(DIRECT_MODE_ERROR);
            break;
        case STREAM4:
            DMA1_Stream4_AppEventCallback(DIRECT_MODE_ERROR);
            break;
        case STREAM5:
            DMA1_Stream5_AppEventCallback(DIRECT_MODE_ERROR);
            break;
        case STREAM6:
            DMA1_Stream6_AppEventCallback(DIRECT_MODE_ERROR);
            break;
        case STREAM7:
            DMA1_Stream7_AppEventCallback(DIRECT_MODE_ERROR);
            break;
        default:
            break;
        }
    }
    else if(pDMAx == DMA2){
        switch(Stream_Num){
        case STREAM0:
            DMA2_Stream0_AppEventCallback(DIRECT_MODE_ERROR);
            break;
        case STREAM1:
            DMA2_Stream1_AppEventCallback(DIRECT_MODE_ERROR);
            break;
        case STREAM2:
            DMA2_Stream2_AppEventCallback(DIRECT_MODE_ERROR);
            break;
        case STREAM3:
            DMA2_Stream3_AppEventCallback(DIRECT_MODE_ERROR);
            break;
        case STREAM4:
            DMA2_Stream4_AppEventCallback(DIRECT_MODE_ERROR);
            break;
        case STREAM5:
            DMA2_Stream5_AppEventCallback(DIRECT_MODE_ERROR);
            break;
        case STREAM6:
            DMA2_Stream6_AppEventCallback(DIRECT_MODE_ERROR);
            break;
        case STREAM7:
            DMA2_Stream7_AppEventCallback(DIRECT_MODE_ERROR);
            break;
        default:
            break;
        }
    }
    else{
        /* do nothing */
    }
}

void FIFO_Error_Callback(DMA_RegDef_t* pDMAx, DMA_Stream_Num_t Stream_Num){

    if(pDMAx == DMA1){
        switch(Stream_Num){
        case STREAM0:
            DMA1_Stream0_AppEventCallback(FIFO_ERROR);
            break;
        case STREAM1:
            DMA1_Stream1_AppEventCallback(FIFO_ERROR);
            break;
        case STREAM2:
            DMA1_Stream2_AppEventCallback(FIFO_ERROR);
            break;
        case STREAM3:
            DMA1_Stream3_AppEventCallback(FIFO_ERROR);
            break;
        case STREAM4:
            DMA1_Stream4_AppEventCallback(FIFO_ERROR);
            break;
        case STREAM5:
            DMA1_Stream5_AppEventCallback(FIFO_ERROR);
            break;
        case STREAM6:
            DMA1_Stream6_AppEventCallback(FIFO_ERROR);
            break;
        case STREAM7:
            DMA1_Stream7_AppEventCallback(FIFO_ERROR);
            break;
        default:
            break;
        }
    }
    else if(pDMAx == DMA2){
        switch(Stream_Num){
        case STREAM0:
            DMA2_Stream0_AppEventCallback(FIFO_ERROR);
            break;
        case STREAM1:
            DMA2_Stream1_AppEventCallback(FIFO_ERROR);
            break;
        case STREAM2:
            DMA2_Stream2_AppEventCallback(FIFO_ERROR);
            break;
        case STREAM3:
            DMA2_Stream3_AppEventCallback(FIFO_ERROR);
            break;
        case STREAM4:
            DMA2_Stream4_AppEventCallback(FIFO_ERROR);
            break;
        case STREAM5:
            DMA2_Stream5_AppEventCallback(FIFO_ERROR);
            break;
        case STREAM6:
            DMA2_Stream6_AppEventCallback(FIFO_ERROR);
            break;
        case STREAM7:
            DMA2_Stream7_AppEventCallback(FIFO_ERROR);
            break;
        default:
            break;
        }
    }
    else{
        /* do nothing */
    }
}
