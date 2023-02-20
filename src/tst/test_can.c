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
#include "rcc_driver.h"
#include "flash_driver.h"
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

/**
 * @brief Function for setting the system clock and the frequency of the CAN peripheral
 * @return void.
 *
 * @note System clock is set to 50MHz and APB1 peripheral clock is set to 25MHz.
 */
static void CAN1_SetClk(void);

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void CAN1_Config(void){

    CAN_Handle_t CAN_Handler = {0};
    CAN_Filter_t CAN_Filter = {0};

    CAN_Handler.pCANx = CAN1;
    CAN_Handler.CAN_Config.CAN_Mode = CAN_MODE_LOOPBACK;
    CAN_Handler.CAN_Config.CAN_AutoBusOff = CAN_ABOM_DISABLE;
    CAN_Handler.CAN_Config.CAN_AutoRetransmission = CAN_NART_ON;
    CAN_Handler.CAN_Config.CAN_AutoWakeup = CAN_AUTO_WAKEUP_OFF;
    CAN_Handler.CAN_Config.CAN_ReceiveFifoLocked = CAN_RX_FIFO_NOT_LOCKED;
    CAN_Handler.CAN_Config.CAN_TimeTriggerMode = CAN_TTCM_DISABLE;
    CAN_Handler.CAN_Config.CAN_TxFifoPriority = CAN_TXFP_ID;

    /* The time register is set to work at 500Kbps */
    CAN_Handler.CAN_Config.CAN_Prescalar = 5;
    CAN_Handler.CAN_Config.CAN_SyncJumpWidth = 1;
    CAN_Handler.CAN_Config.CAN_TimeSeg1 = 8;
    CAN_Handler.CAN_Config.CAN_TimeSeg2 = 1;

    /* Set filter parameters */
    CAN_Filter.FilterNumber = 0;
    CAN_Filter.Mode = CAN_FILTER_ID_MASK_MODE;
    CAN_Filter.Scale = CAN_FILTER_32_BIT_SCALE;
    CAN_Filter.FIFO = CAN_FILTER_FIFO_0;
    CAN_Filter.IdentifierLR = 0x0000;
    CAN_Filter.IdentifierHR = 0x0000;
    CAN_Filter.MaskLR = 0x0000;
    CAN_Filter.MaskHR = 0x0000;

    CAN1_SetClk();
    CAN1_GPIOInit();
    (void)CAN_Init(&CAN_Handler);
    (void)CAN_SetFilter(&CAN_Filter);
}

void CAN1_Send(void){

    uint8_t message[5] = {'H', 'e', 'l', 'l', 'o'};
    CAN_TxHeader_t tx_header = {0};

    tx_header.StId = 0x65D;
    tx_header.IDE = CAN_STDI;
    tx_header.RTR = CAN_DATA_FRAME;
    tx_header.DLC = 5;

    CAN_AddTxMsg(CAN1, &tx_header, message, CAN_MAILBOX_0);
    CAN_AddTxMsg(CAN1, &tx_header, message, CAN_MAILBOX_1);
    CAN_AddTxMsg(CAN1, &tx_header, message, CAN_MAILBOX_2);
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

    /* CAN1 RX */
    CANPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
    GPIO_Init(&CANPins);
}

static void CAN1_SetClk(void){

    RCC_Config_t RCC_Cfg = {0};

    /* Set FLASH latency according to clock frequency (see reference manual) */
    Flash_SetLatency(1);

    /* Set configuration */
    RCC_Cfg.clk_source = RCC_CLK_SOURCE_PLL_P;
    RCC_Cfg.pll_source = PLL_SOURCE_HSE;
    RCC_Cfg.ahb_presc = AHB_NO_PRESC;
    RCC_Cfg.apb1_presc = APB1_PRESC_2;
    RCC_Cfg.apb2_presc = APB2_NO_PRESC;
    RCC_Cfg.pll_n = 50;
    RCC_Cfg.pll_m = 4;
    RCC_Cfg.pll_p = PLL_P_2;
    /* Set clock */
    RCC_SetSystemClock(RCC_Cfg);
}