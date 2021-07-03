/*****************************************************************************************************
* FILENAME :        test.c
*
* DESCRIPTION :
*       File containing configuring function for testing drivers.
*
* PUBLIC FUNCTIONS :
*       void    delay(void)
*       void    LED_GPIOInit(void)
*       void    Button_GPIOInit(void)
*       void    SPI2_Config(void)
*       void    SPI2_SendHello(void)
*       void    SPI_IRQActions(void)
*       void    SPI_SendCmds(void)
*       void    I2C1_Config(void)
*       void    I2C1_SendHello(void)
*
* NOTES :
*       For further information about functions refer to the corresponding header file.
*
**/

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32f446xx.h"
#include "gpio_driver.h"
#include "spi_driver.h"
#include "i2c_driver.h"
#include "test.h"

#define SIZE_ID             11
#define I2C_SLAVE_ADDRESS   0x68

static char rx_buffer[500];
volatile uint8_t rx_stop = 0;
volatile uint8_t read_byte;
SPI_Handle_t SPI2Handle;
I2C_Handle_t I2C1Handle;

/*****************************************************************************************************/
/*                                       Static Function Prototypes                                  */
/*****************************************************************************************************/

/**
 * @fn SPI_CheckAck
 *
 * @brief function to check the ack received from arduino SPI test.
 *
 * @param[in] ack the acknowledge of the communication protocol for testing.
 *
 * @return 1 if ack format is OK and 0 if it is not OK.
 */
static uint8_t SPI_CheckAck(uint8_t ack);

/**
 * @fn SPI2_GPIOInit
 *
 * @brief function to initialize GPIO port for the SPI2 peripheral.
 *
 * @param[in] void
 *
 * @return void
 *
 * @note
 *      PB14 -> SPI2_MISO
 *      PB15 -> SPI2_MOSI
 *      PB13 -> SPI2_SCLK
 *      PB12 -> SPI2_NSS
 *      Alt function mode -> 5
 */
static void SPI2_GPIOInit(void);

/**
 * @fn SPI2_GPIO_IntPinInit
 *
 * @brief function to initialize GPIO pin which SPI issues data available interrupt.
 *
 * @param[in] void
 *
 * @return void
 */
static void SPI2_GPIO_IntPinInit(void);

/**
 * @fn SPI2_Init
 *
 * @brief function to initialize SPI2 peripheral.
 *
 * @param[in] pSPI_Handle.
 *
 * @return void
 */
static void SPI2_Init(SPI_Handle_t* pSPI_Handle);

/**
 * @fn SPI2_SetPinArd
 *
 * @brief function to send command for setting PIN to arduino through SPI2 peripheral.
 *
 * @param[in] void
 *
 * @return void
 */
static void SPI2_SetPinArd(void);

/**
 * @fn SPI2_ReadANArd
 *
 * @brief function to send command for reading analog input of arduino through SPI2 peripheral.
 *
 * @param[in] void
 *
 * @return void
 */
static void SPI2_ReadANArd(void);

/**
 * @fn SPI2_ReadPinArd
 *
 * @brief function to send command for reading pin status of arduino through SPI2 peripheral.
 *
 * @param[in] void
 *
 * @return void
 */
static void SPI2_ReadPinArd(void);

/**
 * @fn SPI2_PrintArd
 *
 * @brief function to send command for printing a message of arduino through SPI2 peripheral.
 *
 * @param[in] void
 *
 * @return void
 */
static void SPI2_PrintArd(void);

/**
 * @fn SPI2_PrintArd
 *
 * @brief function to send command for reading the ID of arduino through SPI2 peripheral.
 *
 * @param[in] void
 *
 * @return void
 */
static void SPI2_ReadIDArd(void);

/**
 * @fn I2C_GPIOInit
 *
 * @brief function to initialize GPIO port for the I2C peripheral.
 *
 * @param[in] void
 *
 * @return void
 *
 * @note
 *      PB8 -> I2C_SCL
 *      PB9 -> I2C_SDA
 *      Alt function mode -> 4
 */
static void I2C1_GPIOInit(void);

/**
 * @fn I2C_Init
 *
 * @brief function to initialize I2C peripheral.
 *
 * @param[in] pI2C_Handle.
 *
 * @return void
 */
static void I2C1_Init(I2C_Handle_t* pI2C_Handle);

/*****************************************************************************************************/
/*                                       Public API Definitions                                      */
/*****************************************************************************************************/

void delay(void){
    for(uint32_t i = 0; i < 500000; i++);
}

void LED_GPIOInit(void){

    GPIO_Handle_t GpioLed;

    memset(&GpioLed, 0, sizeof(GpioLed));

    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULL;

    GPIO_Init(&GpioLed);
}

void Button_GPIOInit(void){

    GPIO_Handle_t GpioBtn;

    memset(&GpioBtn, 0, sizeof(GpioBtn));

    GpioBtn.pGPIOx = GPIOC;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;

    GPIO_Init(&GpioBtn);
}

void SPI2_Config(void){

    /* Pin interrupt for SPI with arduino */
    SPI2_GPIO_IntPinInit();

    /* SPI2 configuration */
    SPI2_GPIOInit();
    SPI2_Init(&SPI2Handle);

    /* Enable the SPI2 SSOE for enabling the NSS output */
    /* The NSS pin is managed by the HW */
    SPI_SSOECfg(SPI2, ENABLE);

    /* Enable SPI2 interrupt */
    SPI_IRQConfig(IRQ_NO_SPI2, ENABLE);
}

void SPI2_SendHello(void){

    char user_data[] = "Hello world";
    uint8_t data_len = strlen(user_data);

    /* Enable the SPI2 peripheral */
    SPI_Enable(SPI2, ENABLE);

    /* Send length information */
    SPI_SendData(SPI2, &data_len, 1);

    /* Send data */
    SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

    /* Confirm SPI is not busy */
    while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

    /* Disable the SPI2 peripheral */
    SPI_Enable(SPI2, DISABLE);
}

void SPI_SendCmds(void){

    SPI2_ReadPinArd();
    delay();
    SPI2_SetPinArd();
    delay();
    SPI2_ReadANArd();
    delay();
    SPI2_ReadPinArd();
    delay();
    SPI2_PrintArd();
    delay();
    SPI2_ReadIDArd();
}

void SPI_IRQActions(void){

    uint8_t dummy = 0xFF;

    GPIO_IRQHandling(GPIO_PIN_NO_9);
    GPIO_IRQConfig(IRQ_NO_EXTI9_5, DISABLE);

    rx_stop = 0;

    /* Enable the SPI2 peripheral */
    SPI_Enable(SPI2, ENABLE);

    while(!rx_stop){
        /* fetch the data from the SPI peripheral byte by byte in interrupt mode */
        while(SPI_SendDataIT(&SPI2Handle, &dummy, 1) == SPI_BUSY_IN_TX);
        while(SPI_ReceiveDataIT(&SPI2Handle, (uint8_t*)&read_byte, 1) == SPI_BUSY_IN_RX);
    }

    /* Confirm SPI is not busy */
    while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

    /* Disable the SPI2 peripheral */
    SPI_Enable(SPI2, DISABLE);

    printf("Rx data = %s\n", rx_buffer);

    GPIO_IRQConfig(IRQ_NO_EXTI9_5, ENABLE);
}

void I2C1_Config(void){

    /* I2C1 configuration */
    I2C1_GPIOInit();
    I2C1_Init(&I2C1Handle);
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

/*****************************************************************************************************/
/*                               Weak Function Overwrite Definitions                                 */
/*****************************************************************************************************/

void SPI2_Handler(void){

    SPI_IRQHandling(&SPI2Handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t* pSPI_Handle, uint8_t app_event){

    static uint32_t i = 0;

    /* In the RX complete event, copy data in to rcv buffer. '\0' indicates end of message(rcvStop = 1) */
    if(app_event == SPI_EVENT_RX_CMPLT){
        rx_buffer[i++] = read_byte;
        if((read_byte == '\0') || (i == 500)){
            rx_stop = 1;
            rx_buffer[i-1] = '\0';
            i = 0;
        }
    }
}

/*****************************************************************************************************/
/*                                       Static Function Definitions                                 */
/*****************************************************************************************************/

static uint8_t SPI_CheckAck(uint8_t ack){

    if(ack == 0xF5){
        return 1;
    }

    return 0;
}

static void SPI2_GPIOInit(void){

    GPIO_Handle_t SPIPins;

    memset(&SPIPins, 0, sizeof(SPIPins));

    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULL;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    /* SLCK */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    /* MOSI */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);

    /* MISO */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPIPins);

    /* NSS */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);
}

static void SPI2_GPIO_IntPinInit(void){

    GPIO_Handle_t SPIIntPin;

    memset(&SPIIntPin, 0, sizeof(SPIIntPin));

    SPIIntPin.pGPIOx = GPIOC;
    SPIIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    SPIIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    SPIIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    SPIIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_Init(&SPIIntPin);

    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIORITY15);
    GPIO_IRQConfig(IRQ_NO_EXTI9_5, ENABLE);
}

static void SPI2_Init(SPI_Handle_t* pSPI_Handle){

    memset(pSPI_Handle, 0, sizeof(*pSPI_Handle));

    pSPI_Handle->pSPIx = SPI2;
    pSPI_Handle->SPIConfig.SPI_BusConfig = SPI_BUS_CFG_FD;
    pSPI_Handle->SPIConfig.SPI_DeviceMode = SPI_DEV_MODE_MASTER;
    pSPI_Handle->SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
    pSPI_Handle->SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    pSPI_Handle->SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    pSPI_Handle->SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    pSPI_Handle->SPIConfig.SPI_SSM = SPI_SSM_DI;

    SPI_Init(pSPI_Handle);
}

static void SPI2_SetPinArd(void){

    uint8_t dummy_write = 0xFF;
    uint8_t dummy_read = 0;
    uint8_t ack_byte = 0;
    uint8_t cmd_code = 0;
    uint8_t args[2] = {0};

    /* Enable the SPI2 peripheral */
    SPI_Enable(SPI2, ENABLE);

    /* CMD_LED_CTRL <pin no(1)> <value(1)> */
    cmd_code = CMD_LED_CTRL;
    SPI_SendData(SPI2, &cmd_code, 1);
    /* Do dummy read to clear off the RXNE */
    SPI_ReceiveData(SPI2, &dummy_read, 1);
    /* Send dummy bits to fetch the response from the slave */
    SPI_SendData(SPI2, &dummy_write, 1);
    /* Read the ack */
    SPI_ReceiveData(SPI2, &ack_byte, 1);
    /* Verify the acknowledge response */
    if(SPI_CheckAck(ack_byte)){
        args[0] = ARD_GPIO_PIN9;
        args[1] = PIN_ON;
        /* Send command */
        SPI_SendData(SPI2, args, 2);
    }

    /* Confirm SPI is not busy */
    while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

    /* Disable the SPI2 peripheral */
    SPI_Enable(SPI2, DISABLE);
}

static void SPI2_ReadANArd(void){

    uint8_t dummy_write = 0xFF;
    uint8_t dummy_read = 0;
    uint8_t ack_byte = 0;
    uint8_t cmd_code = 0;
    uint8_t args = 0;
    uint8_t analog_read = 0;

    /* Enable the SPI2 peripheral */
    SPI_Enable(SPI2, ENABLE);

    /* CMD_SENSOR_READ <pin no(1)> */
    cmd_code = CMD_SENSOR_READ;
    SPI_SendData(SPI2, &cmd_code, 1);
    /* Do dummy read to clear off the RXNE */
    SPI_ReceiveData(SPI2, &dummy_read, 1);
    /* Send dummy bits to fetch the response from the slave */
    SPI_SendData(SPI2, &dummy_write, 1);
    /* Read the ack */
    SPI_ReceiveData(SPI2, &ack_byte, 1);
    /* Verify the acknowledge response */
    if(SPI_CheckAck(ack_byte)){
        args = ARD_AN_PIN0;
        /* Send command */
        SPI_SendData(SPI2, &args, 1);
        /* Do dummy read to clear off the RXNE */
        SPI_ReceiveData(SPI2, &dummy_read, 1);
        /* Insert delay to give some time to be ready the slave */
        delay();
        /* Send dummy bits to fetch the response from the slave */
        SPI_SendData(SPI2, &dummy_write, 1);
        /* Read analog data */
        SPI_ReceiveData(SPI2, &analog_read, 1);

        printf("Analog read: %d\n", analog_read);
    }

    /* Confirm SPI is not busy */
    while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

    /* Disable the SPI2 peripheral */
    SPI_Enable(SPI2, DISABLE);
}

static void SPI2_ReadPinArd(void){

    uint8_t dummy_write = 0xFF;
    uint8_t dummy_read = 0;
    uint8_t ack_byte = 0;
    uint8_t cmd_code = 0;
    uint8_t args = 0;
    uint8_t pin_status_read = 0;

    /* Enable the SPI2 peripheral */
    SPI_Enable(SPI2, ENABLE);

    /* CMD_LED_READ <pin no(1)> */
    cmd_code = CMD_LED_READ;
    SPI_SendData(SPI2, &cmd_code, 1);
    /* Do dummy read to clear off the RXNE */
    SPI_ReceiveData(SPI2, &dummy_read, 1);
    /* Send dummy bits to fetch the response from the slave */
    SPI_SendData(SPI2, &dummy_write, 1);
    /* Read the ack */
    SPI_ReceiveData(SPI2, &ack_byte, 1);
    /* Verify the acknowledge response */
    if(SPI_CheckAck(ack_byte)){
        args = ARD_GPIO_PIN9;
        /* Send command */
        SPI_SendData(SPI2, &args, 1);
        /* Do dummy read to clear off the RXNE */
        SPI_ReceiveData(SPI2, &dummy_read, 1);
        /* Insert delay to give some time to be ready the slave */
        delay();
        /* Send dummy bits to fetch the response from the slave */
        SPI_SendData(SPI2, &dummy_write, 1);
        /* Read response data */
        SPI_ReceiveData(SPI2, &pin_status_read, 1);

        printf("Pin status: %d\n", pin_status_read);
    }

    /* Confirm SPI is not busy */
    while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

    /* Disable the SPI2 peripheral */
    SPI_Enable(SPI2, DISABLE);
}

static void SPI2_PrintArd(void){

    uint8_t user_data[] = "Hello from nucleo board to arduino";
    uint8_t data_len = strlen((char*)user_data);
    uint8_t dummy_write = 0xFF;
    uint8_t dummy_read = 0;
    uint8_t ack_byte = 0;
    uint8_t cmd_code = 0;
    uint8_t args[2] = {0};

    /* Enable the SPI2 peripheral */
    SPI_Enable(SPI2, ENABLE);

    /* CMD_PRINT <len(2)> <message(len)> */
    cmd_code = CMD_PRINT;
    SPI_SendData(SPI2, &cmd_code, 1);
    /* Do dummy read to clear off the RXNE */
    SPI_ReceiveData(SPI2, &dummy_read, 1);
    /* Send dummy bits to fetch the response from the slave */
    SPI_SendData(SPI2, &dummy_write, 1);
    /* Read the ack */
    SPI_ReceiveData(SPI2, &ack_byte, 1);
    /* Verify the acknowledge response */
    if(SPI_CheckAck(ack_byte)){
        args[0] = data_len;
        /* Send command */
        SPI_SendData(SPI2, args, 1);
        /* Do dummy read to clear off the RXNE */
        SPI_ReceiveData(SPI2, &dummy_read, 1);
        /* Insert delay */
        delay();
        /* Send message */
        for(uint8_t i = 0; i < args[0]; i++){
            SPI_SendData(SPI2, &user_data[i], 1);
            SPI_ReceiveData(SPI2, &dummy_read, 1);
        }
    }

    /* Confirm SPI is not busy */
    while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

    /* Disable the SPI2 peripheral */
    SPI_Enable(SPI2, DISABLE);
}

static void SPI2_ReadIDArd(void){

    uint8_t dummy_write = 0xFF;
    uint8_t dummy_read = 0;
    uint8_t ack_byte = 0;
    uint8_t cmd_code = 0;
    uint8_t id[SIZE_ID] = {0};

    /* Enable the SPI2 peripheral */
    SPI_Enable(SPI2, ENABLE);

    /* CMD_ID_READ */
    cmd_code = CMD_ID_READ;
    SPI_SendData(SPI2, &cmd_code, 1);
    /* Do dummy read to clear off the RXNE */
    SPI_ReceiveData(SPI2, &dummy_read, 1);
    /* Send dummy bits to fetch the response from the slave */
    SPI_SendData(SPI2, &dummy_write, 1);
    /* Read the ack */
    SPI_ReceiveData(SPI2, &ack_byte, 1);
    /* Verify the acknowledge response */
    if(SPI_CheckAck(ack_byte)){
        /* Read ID from the slave */
        for(int8_t i = 0; i < (SIZE_ID-1); i++){
            /* Send dummy bits to fetch the response from the slave */
            SPI_SendData(SPI2, &dummy_write, 1);
            SPI_ReceiveData(SPI2, &id[i], 1);
        }
        id[SIZE_ID-1] = '\0';
        printf("ID read: %s\n", id);
    }

    /* Confirm SPI is not busy */
    while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

    /* Disable the SPI2 peripheral */
    SPI_Enable(SPI2, DISABLE);
}

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
