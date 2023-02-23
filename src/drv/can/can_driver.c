/********************************************************************************************************//**
* @file can_driver.c
*
* @brief File containing the APIs for configuring the CAN peripheral.
*
* Public Functions:
*       - uint8_t CAN_Init(CAN_Handle_t* pCAN_Handle)
*       - void CAN_DeInit(CAN_RegDef_t* pCANx)
*       - void CAN_PerClkCtrl(CAN_RegDef_t* pCANx, uint8_t en_or_di)
*       - uint8_t CAN_AddTxMsg(CAN_RegDef_t* pCANx, CAN_TxHeader_t* pTxHeader, uint8_t* msg, uint32_t mailbox)
*       - uint8_t CAN_TxMsgPending(CAN_RegDef_t* pCANx, uint32_t mailbox)
*       - uint8_t CAN_SetFilter(CAN_Filter_t* filter)
*       - uint8_t CAN_GetRxMsg(CAN_RegDef_t* pCANx, CAN_RxMessage_t* pRxMessage, uint8_t FIFO_number)
*       - void CAN_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       - void CAN_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
*       - uint8_t CAN_InterruptsEnable(CAN_RegDef_t* pCANx, uint32_t irq_flags)
*       - uint8_t CAN_InterruptsDisable(CAN_RegDef_t* pCANx, uint32_t irq_flags)
*       - void CAN_Tx_IRQHandling(CAN_RegDef_t* pCANx)
*       - void CAN_Rx0_IRQHandling(CAN_RegDef_t* pCANx)
*       - void CAN_Rx1_IRQHandling(CAN_RegDef_t* pCANx)
*       - void CAN_SCE_IRQHandling(CAN_RegDef_t* pCANx)
*       - void CAN_ApplicationEventCallback(CAN_RegDef_t* pCANx, can_app_event_t app_event)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include "can_driver.h"
#include "stm32f446xx.h"
#include <stdint.h>
#include <stddef.h>

/***********************************************************************************************************/
/*                                       Static Function Prototypes                                        */
/***********************************************************************************************************/

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

uint8_t CAN_Init(CAN_Handle_t* pCAN_Handle){

    uint32_t temp = 0;

    /* Initial check of parameters */
    if((pCAN_Handle->CAN_Config.CAN_SyncJumpWidth < 1) || (pCAN_Handle->CAN_Config.CAN_SyncJumpWidth > 4)){
        return 1;
    }
    if((pCAN_Handle->CAN_Config.CAN_Prescalar < 1) || (pCAN_Handle->CAN_Config.CAN_Prescalar > 0x400)){
        return 1;
    }
    if((pCAN_Handle->CAN_Config.CAN_TimeSeg1 < 1) || (pCAN_Handle->CAN_Config.CAN_TimeSeg1 > 0x10)){
        return 1;
    }
    if((pCAN_Handle->CAN_Config.CAN_TimeSeg2 < 1) || (pCAN_Handle->CAN_Config.CAN_TimeSeg2 > 8)){
        return 1;
    }

    /* Enable the peripheral clock */
    CAN_PerClkCtrl(pCAN_Handle->pCANx, ENABLE);

    /* Enter in initialization mode */
    pCAN_Handle->pCANx->MCR |= (1 << CAN_MCR_INRQ);
    while(!(pCAN_Handle->pCANx->MSR & (1 << CAN_MSR_INAK)));

    /* Set master control register */
    temp |= ((pCAN_Handle->CAN_Config.CAN_TimeTriggerMode << CAN_MCR_TTCM) |
            (pCAN_Handle->CAN_Config.CAN_AutoBusOff << CAN_MCR_ABOM) |
            (pCAN_Handle->CAN_Config.CAN_AutoWakeup << CAN_MCR_AWUM) |
            (pCAN_Handle->CAN_Config.CAN_AutoRetransmission << CAN_MCR_NART) |
            (pCAN_Handle->CAN_Config.CAN_ReceiveFifoLocked << CAN_MCR_RFLM) |
            (pCAN_Handle->CAN_Config.CAN_TxFifoPriority << CAN_MCR_TXFP));
    /* Clean register before write taking into account the INRQ bit */
    pCAN_Handle->pCANx->MCR &= ~(0xFFFFFFFE);
    pCAN_Handle->pCANx->MCR |= temp;

    /* Set bit timing register */
    temp = 0;
    temp |= ((pCAN_Handle->CAN_Config.CAN_Mode << CAN_BTR_LBKM) |
            ((pCAN_Handle->CAN_Config.CAN_SyncJumpWidth - 1) << CAN_BTR_SJW) |
            ((pCAN_Handle->CAN_Config.CAN_Prescalar - 1) << CAN_BTR_BRP) |
            ((pCAN_Handle->CAN_Config.CAN_TimeSeg1 - 1) << CAN_BTR_TS1) |
            ((pCAN_Handle->CAN_Config.CAN_TimeSeg2 - 1) << CAN_BTR_TS2));
    pCAN_Handle->pCANx->BTR &= ~(0xFFFFFFFF);
    pCAN_Handle->pCANx->BTR = temp;

    /* Exit from initialization mode */
    pCAN_Handle->pCANx->MCR &= ~(1 << CAN_MCR_INRQ);
    while(pCAN_Handle->pCANx->MSR & (1 << CAN_MSR_INAK));

    return 0;
}

void CAN_DeInit(CAN_RegDef_t* pCANx){

    if(pCANx == CAN1){
        CAN1_REG_RESET();
    }
    else if(pCANx == CAN2){
        CAN2_REG_RESET();
    }
    else{
        /* do nothing */
    }
}

void CAN_PerClkCtrl(CAN_RegDef_t* pCANx, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        if(pCANx == CAN1){
            CAN1_PCLK_EN();
        }
        else if(pCANx == CAN2){
            CAN2_PCLK_EN();
        }
        else{
            /* do nothing */
        }
    }
    else{
        if(pCANx == CAN1){
            CAN1_PCLK_DI();
        }
        else if(pCANx == CAN2){
            CAN2_PCLK_DI();
        }
        else{
            /* do nothing */
        }
    }
}

uint8_t CAN_AddTxMsg(CAN_RegDef_t* pCANx, CAN_TxHeader_t* pTxHeader, uint8_t* msg, uint32_t mailbox){

    uint32_t temp = 0;
    uint32_t* pTIxR = NULL;

    /* Check if the mailbox is free */
    if(!(pCANx->TSR & (mailbox << CAN_TSR_TME0))){
        return 2;
    }

    /* Set the selected mailbox address to manage it */
    switch(mailbox){
        case CAN_MAILBOX_0:
            pTIxR = (uint32_t*)&(pCANx->TI0R);
            break;
        case CAN_MAILBOX_1:
            pTIxR = (uint32_t*)&(pCANx->TI1R);
            break;
        case CAN_MAILBOX_2:
            pTIxR = (uint32_t*)&(pCANx->TI2R);
            break;
        default:
            return 1;
            break;
    }

    /* Set CAN Tx mailbox identifier register */
    *pTIxR &= ~(0xFFFFFFFF);

    if(pTxHeader->IDE == CAN_STDI){
        temp |= (pTxHeader->StId << CAN_TIxR_STID);
    }
    else{
        temp |= (pTxHeader->ExId << CAN_TIxR_EXID);
    }
    temp |= ((pTxHeader->IDE << CAN_TIxR_IDE) |
            (pTxHeader->RTR << CAN_TIxR_RTR));
    *pTIxR = temp;

    /* Set DLC value (TDTxR register has offset 4 bytes from TIxR register) */
    *(pTIxR + 1) &= ~(0x0000000F);
    *(pTIxR + 1) |= (pTxHeader->DLC << CAN_TDTxR_DLC);

    /* Set CAN mailbox data register */
    /* Clear TDLxR register (offset 8 bytes from TIxR register) */
    *(pTIxR + 2) &= ~(0xFFFFFFFF);
    /* Clear TDHxR register (offset 12 bytes from TIxR register) */
    *(pTIxR + 3) &= ~(0xFFFFFFFF);
    /* Set TDLxR register */
    temp = 0;
    temp |= ((msg[0] << 0) |
            (msg[1] << 8) |
            (msg[2] << 16) |
            (msg[3] << 24));
    *(pTIxR + 2) = temp;
    /* Set TDHxR register */
    temp = 0;
    temp |= ((msg[4] << 0) |
            (msg[5] << 8) |
            (msg[6] << 16) |
            (msg[7] << 24));
    *(pTIxR + 3) = temp;

    /* Request transmission */
    *(pTIxR) |= (1 << CAN_TIxR_TXRQ);

    return 0;
}

uint8_t CAN_TxMsgPending(CAN_RegDef_t* pCANx, uint32_t mailbox){

    uint8_t ret = 0;

    if(pCANx->TSR & (mailbox << CAN_TSR_TME0)){
        return 1;
    }

    return ret;
}

uint8_t CAN_SetFilter(CAN_Filter_t* filter){

    uint32_t temp = 0;

    /* Check the filter number is valid */
    if(filter->FilterNumber > 27){
        return 1;
    }

    /* Enter in filter initialization mode */
    CAN1->FMR |= (1 << CAN_FMR_FINIT);

    /* Set filter mode */
    CAN1->FM1R &= ~(1 << filter->FilterNumber);
    CAN1->FM1R |= (filter->Mode << filter->FilterNumber);

    /* Set filter scale */
    CAN1->FS1R &= ~(1 << filter->FilterNumber);
    CAN1->FS1R |= (filter->Scale << filter->FilterNumber);

    /* Set filter FIFO assignment */
    CAN1->FFA1R &= ~(1 << filter->FilterNumber);
    CAN1->FFA1R |= (filter->FIFO << filter->FilterNumber);

    /* Set filter banks */
    /* Clear identifier filter register */
    CAN1->FiRx[2*(filter->FilterNumber)] &= ~(0xFFFFFFFF);
    /* Set identifier filter register */
    temp |= ((filter->IdentifierHR << 16) |
            (filter->IdentifierLR));
    CAN1->FiRx[2*(filter->FilterNumber)] = temp;
    /* Clear mask filter register */
    CAN1->FiRx[2*(filter->FilterNumber) + 1] &= ~(0xFFFFFFFF);
    /* Set mask filter register */
    temp = 0;
    temp |= ((filter->MaskHR << 16) |
            (filter->MaskLR));
    CAN1->FiRx[2*(filter->FilterNumber) + 1] = temp;

    /* Active filters mode */
    CAN1->FMR &= ~(1 << CAN_FMR_FINIT);

    /* Activate filter */
    CAN1->FA1R |= (1 << filter->FilterNumber);

    return 0;
}

uint8_t CAN_GetRxMsg(CAN_RegDef_t* pCANx, CAN_RxMessage_t* pRxMessage, uint8_t FIFO_number){

    uint32_t* pFIFO = NULL;

    /* Check if selected FIFO has a message pending */
    if(FIFO_number == 0){
        if(!(pCANx->RF0R & (0x3 << CAN_RFxR_FMP))){
            return 2;
        }
        pFIFO = (uint32_t*)&(pCANx->RI0R);
    }
    else if(FIFO_number == 1){
        if(!(pCANx->RF1R & (0x3 << CAN_RFxR_FMP))){
            return 2;
        }
        pFIFO = (uint32_t*)&(pCANx->RI1R);
    }
    else{
        /* If FIFO number is not correct return error */
        return 1;
    }

    pRxMessage->IDE = (*pFIFO << CAN_RIxR_IDE);
    pRxMessage->RTR = (*pFIFO << CAN_RIxR_RTR);
    if(pRxMessage->IDE == 0){
        pRxMessage->StId = (*pFIFO << CAN_RIxR_STID);
    }
    else{
        pRxMessage->ExId = (*pFIFO << CAN_RIxR_EXID);
    }
    pRxMessage->DLC = (*(pFIFO + 1) << CAN_RDTxR_DLC);
    pRxMessage->DataLR = *(pFIFO + 2);
    pRxMessage->DataHR = *(pFIFO + 3);

    /* Release the output mailbox */
    if(FIFO_number == 0){
        pCANx->RF0R |= (1 << CAN_RFxR_RFOM);
    }
    else{
        pCANx->RF1R |= (1 << CAN_RFxR_RFOM);
    }

    return 0;
}

void CAN_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        if(IRQNumber <= 31){
            /* Program ISER0 register */
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64){
            /* Program ISER1 register */
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96){
            /* Program ISER2 register */
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
        else{
            /* do nothing */
        }
    }
    else{
        if(IRQNumber <= 31){
            /* Program ICER0 register */
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64){
            /* Program ICER1 register */
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96){
            /* Program ICER2 register */
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
        else{
            /* do nothing */
        }
    }
}

void CAN_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
    /* Find out the IPR register */
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift = (8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift);
}

uint8_t CAN_InterruptsEnable(CAN_RegDef_t* pCANx, uint32_t irq_flags){

    if(irq_flags & 0xFFFC7080){
        return 1;
    }

    pCANx->IER |= irq_flags;

    return 0;
}

uint8_t CAN_InterruptsDisable(CAN_RegDef_t* pCANx, uint32_t irq_flags){

    if(irq_flags & 0xFFFC7080){
        return 1;
    }

    pCANx->IER &= ~irq_flags;

    return 0;
}

void CAN_Tx_IRQHandling(CAN_RegDef_t* pCANx){

    if(pCANx->TSR & (1 << CAN_TSR_RQCP0)){
        CAN_ApplicationEventCallback(pCANx, CAN_TX_REQ_CMPT_M0);
    }
    else if(pCANx->TSR & (1 << CAN_TSR_RQCP1)){
        CAN_ApplicationEventCallback(pCANx, CAN_TX_REQ_CMPT_M1);
    }
    else if(pCANx->TSR & (1 << CAN_TSR_RQCP2)){
        CAN_ApplicationEventCallback(pCANx, CAN_TX_REQ_CMPT_M2);
    }
    else{
        /* do nothing */
    }

}

void CAN_Rx0_IRQHandling(CAN_RegDef_t* pCANx){

    if(pCANx->RF0R & (0x3 << CAN_RFxR_FMP)){
        CAN_ApplicationEventCallback(pCANx, CAN_FIFO0_MSG_PEND);
    }
    else if(pCANx->RF0R & (1 << CAN_RFxR_FULL)){
        CAN_ApplicationEventCallback(pCANx, CAN_FIFO0_FULL);
    }
    else if(pCANx->RF0R & (1<< CAN_RFxR_FOVR)){
        CAN_ApplicationEventCallback(pCANx, CAN_FIFO0_OVERRUN);
    }
    else{
        /* do nothing */
    }
}

void CAN_Rx1_IRQHandling(CAN_RegDef_t* pCANx){

    if(pCANx->RF1R & (0x3 << CAN_RFxR_FMP)){
        CAN_ApplicationEventCallback(pCANx, CAN_FIFO1_MSG_PEND);
    }
    else if(pCANx->RF1R & (1 << CAN_RFxR_FULL)){
        CAN_ApplicationEventCallback(pCANx, CAN_FIFO1_FULL);
    }
    else if(pCANx->RF1R & (1<< CAN_RFxR_FOVR)){
        CAN_ApplicationEventCallback(pCANx, CAN_FIFO1_OVERRUN);
    }
    else{
        /* do nothing */
    }
}

void CAN_SCE_IRQHandling(CAN_RegDef_t* pCANx){

    /* Check if the irq is caused by an error */
    if(pCANx->MSR & (1 << CAN_MSR_ERRI)){
        if(pCANx->ESR & (1<< CAN_ESR_EWGF)){
            CAN_ApplicationEventCallback(pCANx, CAN_ERROR_WARNING);
        }
        else if(pCANx->ESR & (1 << CAN_ESR_EPVF)){
            CAN_ApplicationEventCallback(pCANx, CAN_ERROR_PASSIVE);
        }
        else if(pCANx->ESR & (1 << CAN_ESR_BOFF)){
            CAN_ApplicationEventCallback(pCANx, CAN_ERROR_BUSOFF);
        }
        else if(pCANx->ESR & (0x7 << CAN_ESR_LEC)){
            CAN_ApplicationEventCallback(pCANx, CAN_ERROR_CODE);
        }
        else{
            /* do nothing */
        }
    }
    if(pCANx->MSR & (1 << CAN_MSR_SLAKI)){
        CAN_ApplicationEventCallback(pCANx, CAN_SLEEP_ACK);
    }
    else if(pCANx->MSR & (1 << CAN_MSR_WKUI)){
        CAN_ApplicationEventCallback(pCANx, CAN_WAKEUP_IRQ);
    }
    else{
        /* do nothing */
    }
}

/***********************************************************************************************************/
/*                                       Weak Functions                                                    */
/*            This is a weak implementation. The application may override this function                    */
/***********************************************************************************************************/

__attribute__((weak)) void CAN_ApplicationEventCallback(CAN_RegDef_t* pCANx, can_app_event_t app_event){

    /* This is a weak implementation. The application may override this function */
}

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/
