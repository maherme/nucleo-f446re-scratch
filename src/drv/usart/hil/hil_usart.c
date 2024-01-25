/********************************************************************************************************//**
* @file hil_usart.c
*
* @brief File containing the APIs for testing the USART peripheral in the hardware.
*
* Public Functions:
*       - void    USART3_Config(void)
*       - void    USART3_SendHello(void)
*       - void    USART3_TxRx(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include "hil_usart.h"
#include "usart_driver.h"
#include "gpio_driver.h"
#include "cortex_m4.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#if TEST_USART

/** @brief Flag for notifying completed reception */
static uint8_t usart_rx_cplt = RESET;
/** @brief Handler structure for USART peripheral */
USART_Handle_t USART3Handle;

/***********************************************************************************************************/
/*                                       Static Function Prototypes                                        */
/***********************************************************************************************************/

/**
 * @brief Function to initialize USART3 peripheral.
 * @return void.
 */
static void USART3_Init(USART_Handle_t* pUSART_Handle);

/**
 * @brief Function to initialize GPIO port for the USART3 peripheral.
 * @return void.
 *
 * @note
 *      PC10 -> USART3 TX
 *      PC11 -> USART3 RX
 *      Alt function mode -> 7
 */
static void USART3_GPIOInit(void);

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void USART3_Config(void){

    /* USART3 configuration */
    USART3_GPIOInit();
    USART3_Init(&USART3Handle);

    /* USART3 interrupt configuration */
    IRQConfig(IRQ_NO_USART3, ENABLE);
}

void USART3_SendHello(void){

    char user_data[] = "Hello world\n";

    /* Enable the USART3 peripheral */
    USART_Enable(USART3, ENABLE);

    /* Send data */
    USART_SendData(&USART3Handle, (uint8_t*)user_data, strlen(user_data));
    printf("Data transmitted: %s", user_data);

    /* Disable the USART3 peripheral */
    USART_Enable(USART3, DISABLE);
}

void USART3_TxRx(void){

    char user_data[] = "Hello World\n";
    uint8_t rx_buf[32] = {0};

    /* Enable the USART3 peripheral */
    USART_Enable(USART3, ENABLE);

    /* Enable reception in interrupt mode */
    while(USART_ReceiveDataIT(&USART3Handle, rx_buf, strlen(user_data)) != USART_READY);

    /* Send data */
    USART_SendData(&USART3Handle, (uint8_t*)user_data, strlen(user_data));
    printf("Data transmitted: %s", user_data);

    /* Wait until all bytes are received */
    while(usart_rx_cplt != SET);

    /* Set last byte to '\0' */
    rx_buf[strlen(user_data) + 1] = '\0';

    /* Print data received */
    printf("Data received   : %s", rx_buf);

    /* Reset reception completed flag */
    usart_rx_cplt = RESET;

    /* Disable the USART3 peripheral */
    USART_Enable(USART3, DISABLE);
}

/***********************************************************************************************************/
/*                               Weak Function Overwrite Definitions                                       */
/***********************************************************************************************************/

void USART3_Handler(void){

    USART_IRQHandling(&USART3Handle);
}

void USART_ApplicationEventCallback(USART_Handle_t* pUSART_Handle, uint8_t app_event){

    if(app_event == USART_EVENT_RX_CMPLT){
        usart_rx_cplt = SET;
    }
    else if(app_event == USART_EVENT_TX_CMPLT){
        /* do nothing */
    }
    else{
        /* do nothing */
    }
}

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/

static void USART3_Init(USART_Handle_t* pUSART_Handle){

    memset(pUSART_Handle, 0, sizeof(*pUSART_Handle));

    pUSART_Handle->pUSARTx = USART3;
    pUSART_Handle->USART_Config.USART_Baud = USART_STD_BAUD_115200;
    pUSART_Handle->USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    pUSART_Handle->USART_Config.USART_Mode = USART_MODE_TXRX;
    pUSART_Handle->USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    pUSART_Handle->USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    pUSART_Handle->USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

    USART_Init(pUSART_Handle);
}

static void USART3_GPIOInit(void){

    GPIO_Handle_t USARTPins;

    memset(&USARTPins, 0, sizeof(USARTPins));

    USARTPins.pGPIOx = GPIOC;
    USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    USARTPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    USARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

    /* USART3 TX */
    USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
    GPIO_Init(&USARTPins);

    /* USART3 RX */
    USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
    GPIO_Init(&USARTPins);
}

#endif  /* TEST_USART */
