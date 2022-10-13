/********************************************************************************************************//**
* @file test_dma.c
*
* @brief File containing the APIs for testing the DMA peripheral.
*
* Public Functions:
*       - void      DMA1_Config(void)
*       - void      DMA1_USART3_Request(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include <string.h>
#include "dma_driver.h"
#include "usart_driver.h"
#include "gpio_driver.h"

#include <stdio.h>

/** @brief Handler structure for DMA peripheral */
DMA_Handle_t DMA1Handle;
/** @brief Handler structure for DMA stream */
DMA_Stream_Handle_t Stream3Handle;
/** @brief String for sending */
char test_data[] = "Hello World\r\n";

/***********************************************************************************************************/
/*                                       Static Function Prototypes                                        */
/***********************************************************************************************************/

/**
 * @brief Function to initialize DMA1 peripheral.
 * @return void.
 */
static void DMA1_Init(DMA_Handle_t* pDMA_Handle);

/**
 * @brief Function to initialize Stream6 for DMA1 peripheral
 * @return void
 */
static void DMA1_Stream3_Init(DMA_Stream_Handle_t* pStream_Handle);

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

void DMA1_Config(void){

    /** @brief Handler structure for USART peripheral */
    USART_Handle_t USART3Handle;

    /* USART3 configuration */
    USART3_GPIOInit();
    USART3_Init(&USART3Handle);

    /* Enable the USART3 peripheral */
    USART_Enable(USART3, ENABLE);

    DMA1_Init(&DMA1Handle);
    DMA1_Stream3_Init(&Stream3Handle);
}

void DMA1_USART3_Request(void){

    USART_RegDef_t* pUSART3;
    pUSART3 = USART3;

    /* Prepare Stream and DMA for a transmission or retransmission */
    DMA_Stream_Set_NDTR(&Stream3Handle, (uint32_t)sizeof(test_data));
    DMA_Clear_Transfer_Compl_Int_Flag(DMA1Handle.pDMAx, STREAM3);
    DMA_Stream_Enable(&Stream3Handle);

    /* Set DMA enable transmitter bit */
    pUSART3->CR3 |= (1 << USART_CR3_DMAT);
}

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/

static void DMA1_Init(DMA_Handle_t* pDMA_Handle){

    memset(pDMA_Handle, 0, sizeof(*pDMA_Handle));

    pDMA_Handle->pDMAx = DMA1;

    DMA_Init(pDMA_Handle);
}

static void DMA1_Stream3_Init(DMA_Stream_Handle_t* pStream_Handle){

    USART_RegDef_t* pUSART3;
    pUSART3 = USART3;

    memset(pStream_Handle, 0, sizeof(*pStream_Handle));

    pStream_Handle->pStreamx = DMA1_STR3;
    pStream_Handle->Stream_Config.src_addr = (uint32_t)test_data;
    pStream_Handle->Stream_Config.dst_addr = (uint32_t)&pUSART3->DR;
    pStream_Handle->Stream_Config.num_data_register = sizeof(test_data);
    pStream_Handle->Stream_Config.direction = M2P;
    pStream_Handle->Stream_Config.msize = BYTE;
    pStream_Handle->Stream_Config.minc = ENABLE;
    pStream_Handle->Stream_Config.msize = BYTE;
    pStream_Handle->Stream_Config.mode = FIFO_MODE;
    pStream_Handle->Stream_Config.FIFO_thres = FULL;
    pStream_Handle->Stream_Config.circular_mode = DISABLE;
    pStream_Handle->Stream_Config.priority = LOW;
    pStream_Handle->Stream_Config.ch_number = 4;

    DMA_Stream_Init(pStream_Handle);
}

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
