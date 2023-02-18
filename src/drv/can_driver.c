/********************************************************************************************************//**
* @file can_driver.c
*
* @brief File containing the APIs for configuring the CAN peripheral.
*
* Public Functions:
*       - void CAN_Init(CAN_Handle_t* pCAN_Handle)
*       - void CAN_DeInit(CAN_RegDef_t* pCANx)
*       - void CAN_PerClkCtrl(CAN_RegDef_t* pCANx, uint8_t en_or_di)
*       - void CAN_AddTxMsg(CAN_RegDef_t* pCANx, CAN_TxHeader_t* pTxHeader, uint8_t* msg)
*       - uint8_t CAN_TxMsgPending(CAN_RegDef_t* pCANx)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include "can_driver.h"
#include "stm32f446xx.h"
#include <stdint.h>

/***********************************************************************************************************/
/*                                       Static Function Prototypes                                        */
/***********************************************************************************************************/

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void CAN_Init(CAN_Handle_t* pCAN_Handle){

    uint32_t temp = 0;

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
            (pCAN_Handle->CAN_Config.CAN_SyncJumpWidth << CAN_BTR_SJW) |
            (pCAN_Handle->CAN_Config.CAN_Prescalar << CAN_BTR_BRP) |
            (pCAN_Handle->CAN_Config.CAN_TimeSeg1 << CAN_BTR_TS1) |
            (pCAN_Handle->CAN_Config.CAN_TimeSeg2 << CAN_BTR_TS2));
    pCAN_Handle->pCANx->BTR &= ~(0xFFFFFFFF);
    pCAN_Handle->pCANx->BTR = temp;

    /* Exit from initialization mode */
    pCAN_Handle->pCANx->MCR &= ~(1 << CAN_MCR_INRQ);
    while(pCAN_Handle->pCANx->MSR & (1 << CAN_MSR_INAK));
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

void CAN_AddTxMsg(CAN_RegDef_t* pCANx, CAN_TxHeader_t* pTxHeader, uint8_t* msg){

    uint32_t temp = 0;

    /* Set CAN Tx mailbox identifier register */
    pCANx->TI0R &= ~(0xFFFFFFFF);

    if(pTxHeader->IDE == CAN_STDI){
        temp |= (pTxHeader->StId << CAN_TIxR_STID);
    }
    else{
        temp |= (pTxHeader->ExId << CAN_TIxR_EXID);
    }
    temp |= ((pTxHeader->IDE << CAN_TIxR_IDE) |
            (pTxHeader->RTR << CAN_TIxR_RTR));
    pCANx->TI0R = temp;

    /* Set DLC value */
    pCANx->TDT0R &= ~(0x0000000F);
    pCANx->TDT0R |= (pTxHeader->DLC << CAN_TDTxR_DLC);

    /* Set CAN mailbox data register */
    pCANx->TDL0R &= ~(0xFFFFFFFF);
    pCANx->TDH0R &= ~(0xFFFFFFFF);
    temp = 0;
    temp |= ((msg[0] << 0) |
            (msg[1] << 8) |
            (msg[2] << 16) |
            (msg[3] << 24));
    pCANx->TDL0R = temp;
    temp = 0;
    temp |= ((msg[4] << 0) |
            (msg[5] << 8) |
            (msg[6] << 16) |
            (msg[7] << 24));
    pCANx->TDH0R = temp;

    /* Request transmission */
    pCANx->TI0R |= (1 << CAN_TIxR_TXRQ);
}

uint8_t CAN_TxMsgPending(CAN_RegDef_t* pCANx){

    uint8_t ret = 0;

    if(pCANx->TSR & (1 << CAN_TSR_TME0)){
        return 1;
    }

    return ret;
}

/***********************************************************************************************************/
/*                                       Weak Functions                                                    */
/*            This is a weak implementation. The application may override this function                    */
/***********************************************************************************************************/

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/
