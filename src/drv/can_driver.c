/********************************************************************************************************//**
* @file can_driver.c
*
* @brief File containing the APIs for configuring the CAN peripheral.
*
* Public Functions:
*       - void CAN_Init(CAN_Handle_t* pCAN_Handle)
*       - void CAN_DeInit(CAN_RegDef_t* pCANx)
*       - void CAN_PerClkCtrl(CAN_RegDef_t* pCANx, uint8_t en_or_di)
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

/***********************************************************************************************************/
/*                                       Weak Functions                                                    */
/*            This is a weak implementation. The application may override this function                    */
/***********************************************************************************************************/

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/
