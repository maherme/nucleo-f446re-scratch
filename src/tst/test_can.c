/********************************************************************************************************//**
* @file test_can.c
*
* @brief File containing the APIs for testing the CAN peripheral.
*
* Public Functions:
*       - void CAN1_Config(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include "can_driver.h"

/***********************************************************************************************************/
/*                                       Static Function Prototypes                                        */
/***********************************************************************************************************/

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void CAN1_Config(void){

    CAN_Handle_t CAN_Handler = {0};

    CAN_Handler.pCANx = CAN1;
    CAN_Handler.CAN_Config.CAN_Mode = CAN_MODE_NORMAL;
    CAN_Handler.CAN_Config.CAN_AutoBusOff = CAN_ABOM_DISABLE;
    CAN_Handler.CAN_Config.CAN_AutoRetransmission = CAN_NART_ON;
    CAN_Handler.CAN_Config.CAN_AutoWakeup = CAN_AUTO_WAKEUP_OFF;
    CAN_Handler.CAN_Config.CAN_ReceiveFifoLocked = CAN_RX_FIFO_NOT_LOCKED;
    CAN_Handler.CAN_Config.CAN_TimeTriggerMode = CAN_TTCM_DISABLE;
    CAN_Handler.CAN_Config.CAN_TxFifoPriority = CAN_TXFP_ID;

    CAN_Handler.CAN_Config.CAN_Prescalar = 5;
    CAN_Handler.CAN_Config.CAN_SyncJumpWidth = 0;
    CAN_Handler.CAN_Config.CAN_TimeSeg1 = 7;
    CAN_Handler.CAN_Config.CAN_TimeSeg2 = 0;

    CAN_Init(&CAN_Handler);
}

/***********************************************************************************************************/
/*                               Weak Function Overwrite Definitions                                       */
/***********************************************************************************************************/

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/