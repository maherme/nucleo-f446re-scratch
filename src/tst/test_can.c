/********************************************************************************************************//**
* @file test_can.c
*
* @brief File containing the APIs for testing the CAN peripheral.
*
* Public Functions:
*       - void CAN1_Config(void)
*       - void CAN1_Send(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include "can_driver.h"
#include "gpio_driver.h"
#include <stdint.h>
#include <string.h>

/***********************************************************************************************************/
/*                                       Static Function Prototypes                                        */
/***********************************************************************************************************/

/**
 * @brief Function to initialize GPIO port for the CAN1 peripheral.
 * @return void.
 *
 * @note
 *      PA11 -> CAN1 RX
 *      PA12 -> CAN1 TX
 *      Alt function mode -> 9
 */
static void CAN1_GPIOInit(void);

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

    CAN1_GPIOInit();
    CAN_Init(&CAN_Handler);
}

void CAN1_Send(void){

    uint8_t message[5] = {'H', 'e', 'l', 'l', 'o'};
    CAN_TxHeader_t tx_header = {0};

    tx_header.StId = 0x65D;
    tx_header.IDE = CAN_STDI;
    tx_header.RTR = CAN_DATA_FRAME;
    tx_header.DLC = 5;

    CAN_AddTxMsg(CAN1, &tx_header, message);
    while(CAN_TxMsgPending(CAN1));
}

/***********************************************************************************************************/
/*                               Weak Function Overwrite Definitions                                       */
/***********************************************************************************************************/

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/

static void CAN1_GPIOInit(void){

    GPIO_Handle_t CANPins;

    memset(&CANPins, 0, sizeof(CANPins));

    CANPins.pGPIOx = GPIOA;
    CANPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    CANPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    CANPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULL;
    CANPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    CANPins.GPIO_PinConfig.GPIO_PinAltFunMode = 9;

    /* CAN1 TX */
    CANPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&CANPins);

    /* USART3 RX */
    CANPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
    GPIO_Init(&CANPins);
}