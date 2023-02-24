/********************************************************************************************************//**
* @file test_i2c.c
*
* @brief File containing the APIs for testing the I2C peripheral.
*
* Public Functions:
*       - void    I2C1_Config(void)
*       - void    I2C1_SendHello(void)
*       - void    I2C1_SendCmd(void)
*       - void    I2C1_SendCmdIT(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include "test_i2c.h"
#include "i2c_driver.h"
#include "gpio_driver.h"
#include "cortex_m4.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/** @brief Slave address for I2C peripheral */
#define I2C_SLAVE_ADDRESS   0x68
/** @brief Set I2C_MASTER to 1 for master mode or to 0 for slave mode */
#define I2C_MASTER          1

/** @brief Flag for completed reception */
static uint8_t i2c_rx_cplt = RESET;
/** @brief Handler structure for I2C peripheral */
I2C_Handle_t I2C1Handle;

/***********************************************************************************************************/
/*                                       Static Function Prototypes                                        */
/***********************************************************************************************************/

/**
 * @brief Function to initialize GPIO port for the I2C peripheral.
 * @return void.
 * @note
 *      PB8 -> I2C_SCL
 *      PB9 -> I2C_SDA
 *      Alt function mode -> 4
 */
static void I2C1_GPIOInit(void);

/**
 * @brief Function to initialize I2C peripheral.
 * @param[in] pI2C_Handle Is the handler structure for managing the I2C peripheral.
 * @return void.
 */
static void I2C1_Init(I2C_Handle_t* pI2C_Handle);

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void I2C1_Config(void){

    /* I2C1 configuration */
    I2C1_GPIOInit();
    I2C1_Init(&I2C1Handle);

    /* Enable I2C1 interrupt */
    IRQConfig(IRQ_NO_I2C1_EV, ENABLE);
    IRQConfig(IRQ_NO_I2C1_ER, ENABLE);

#if I2C_MASTER == 0
    /* Enable callback events, it is needed in slave mode */
    I2C_SlaveEnCallbackEvents(I2C1, ENABLE);

    /* Enable peripheral, ONLY in slave mode */
    I2C_Enable(I2C1, ENABLE);
#endif

    /* Enable acking */
    I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);
}

void I2C1_SendHello(void){

    char user_data[] = "Hello world";

    /* Enable the I2C1 peripheral */
    I2C_Enable(I2C1, ENABLE);

    /* Send data */
    I2C_MasterSendData(&I2C1Handle, (uint8_t*)user_data, strlen(user_data), I2C_SLAVE_ADDRESS, I2C_DISABLE_SR);

    /* Disable the I2C1 peripheral */
    I2C_Enable(I2C1, DISABLE);
}

void I2C1_SendCmd(void){

    uint8_t rx_buf[32] = {0};
    uint8_t len = 0;
    uint8_t command = 0x51;

    /* Enable the I2C1 peripheral */
    I2C_Enable(I2C1, ENABLE);

    I2C_MasterSendData(&I2C1Handle, &command, 1, I2C_SLAVE_ADDRESS, I2C_ENABLE_SR);
    I2C_MasterReceiveData(&I2C1Handle, &len, 1, I2C_SLAVE_ADDRESS, I2C_ENABLE_SR);

    command = 0x52;
    I2C_MasterSendData(&I2C1Handle, &command, 1, I2C_SLAVE_ADDRESS, I2C_ENABLE_SR);
    I2C_MasterReceiveData(&I2C1Handle, rx_buf, len, I2C_SLAVE_ADDRESS, I2C_ENABLE_SR);
    rx_buf[len + 1] = '\0';
    printf("Data: %s", rx_buf);

    /* Disable the I2C1 peripheral */
    I2C_Enable(I2C1, DISABLE);
}

void I2C1_SendCmdIT(void){

    uint8_t rx_buf[32] = {0};
    uint8_t len = 0;
    uint8_t command = 0x51;

    /* Enable the I2C1 peripheral */
    I2C_Enable(I2C1, ENABLE);

    while(I2C_MasterSendDataIT(&I2C1Handle, &command, 1, I2C_SLAVE_ADDRESS, I2C_ENABLE_SR) != I2C_READY);
    while(I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, I2C_SLAVE_ADDRESS, I2C_ENABLE_SR) != I2C_READY);

    command = 0x52;
    while(I2C_MasterSendDataIT(&I2C1Handle, &command, 1, I2C_SLAVE_ADDRESS, I2C_ENABLE_SR) != I2C_READY);
    while(I2C_MasterReceiveDataIT(&I2C1Handle, rx_buf, len, I2C_SLAVE_ADDRESS, I2C_ENABLE_SR) != I2C_READY);

    i2c_rx_cplt = RESET;
    /* Wait until rx completes */
    while(i2c_rx_cplt != SET);
    rx_buf[len + 1] = '\0';
    printf("Data: %s", rx_buf);
    i2c_rx_cplt = RESET;

    /* Disable the I2C1 peripheral */
    I2C_Enable(I2C1, DISABLE);
}

/***********************************************************************************************************/
/*                               Weak Function Overwrite Definitions                                       */
/***********************************************************************************************************/

void I2C1_EV_Handler(void){

    I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_Handler(void){

    I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t* pI2C_Handle, uint8_t app_event){

    static uint8_t tx_buf[32] = "Nucleo board slave mode";
    static uint8_t cmd_code = 0;
    static uint8_t cnt = 0;

    switch(app_event){
        case I2C_EVENT_TX_CMPLT:
            printf("Tx is completed\n");
            break;
        case I2C_EVENT_RX_CMPLT:
            printf("Rx is completed\n");
            i2c_rx_cplt = SET;
            break;
        case I2C_EVENT_DATA_REQ:
            if(cmd_code == 0x51){
                /* Send length information to the master */
                I2C_SlaveSendData(pI2C_Handle->pI2Cx, strlen((char*)tx_buf));
            }
            else if(cmd_code == 0x52){
                /* Send the contents of tx_buf */
                I2C_SlaveSendData(pI2C_Handle->pI2Cx, tx_buf[cnt++]);
            }
            else{
                /* do nothing */
            }
            break;
        case I2C_EVENT_DATA_RCV:
            /* Data wainting for the slave to read */
            cmd_code = I2C_SlaveReceiveData(pI2C_Handle->pI2Cx);
            break;
        case I2C_ERROR_AF:
#if I2C_MASTER
            printf("Error: acknowledge failure\n");
            I2C_CloseSendData(pI2C_Handle);
            /* Generate the stop condition to release the bus */
            I2C_GenerateStopCondition(I2C1);
#else
            cmd_code = 0xFF;
            cnt = 0;
#endif
            break;
        default:
            break;
    }
}

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/

static void I2C1_GPIOInit(void){

    GPIO_Handle_t I2CPins;

    memset(&I2CPins, 0, sizeof(I2CPins));

    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    /* SCL */
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
    GPIO_Init(&I2CPins);

    /* SDA */
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    GPIO_Init(&I2CPins);
}

static void I2C1_Init(I2C_Handle_t* pI2C_Handle){

    memset(pI2C_Handle, 0, sizeof(*pI2C_Handle));

    pI2C_Handle->pI2Cx = I2C1;
    pI2C_Handle->I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
    pI2C_Handle->I2C_Config.I2C_DeviceAddress = 0x61;
    pI2C_Handle->I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
    pI2C_Handle->I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

    I2C_Init(pI2C_Handle);
}
