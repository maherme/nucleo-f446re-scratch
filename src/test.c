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
*       void    SPI2_GPIOInit(void)
*       void    SPI2_Init(void)
*       void    SPI2_SendHello(void)
*       void    SPI2_SetPinArd(void)
*       void    SPI2_ReadANArd(void)
*       void    SPI2_ReadPinArd(void)
*       void    SPI2_PrintArd(void)
*       void    SPI2_ReadIDArd(void)
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
#include "test.h"

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
static uint8_t SPI_CheckAck(uint8_t ack){

    if(ack == 0xF5){
        return 1;
    }

    return 0;
}

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


void SPI2_GPIOInit(void){

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

void SPI2_Init(void){

    SPI_Handle_t SPI2Handle;

    memset(&SPI2Handle, 0, sizeof(SPI2Handle));

    SPI2Handle.pSPIx = SPI2;
    SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CFG_FD;
    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEV_MODE_MASTER;
    SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

    SPI_Init(&SPI2Handle);
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

void SPI2_SetPinArd(void){

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

void SPI2_ReadANArd(void){

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

        printf("Data read: %d\n", analog_read);
    }

    /* Confirm SPI is not busy */
    while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

    /* Disable the SPI2 peripheral */
    SPI_Enable(SPI2, DISABLE);
}

void SPI2_ReadPinArd(void){

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

        printf("Data read: %d\n", pin_status_read);
    }

    /* Confirm SPI is not busy */
    while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

    /* Disable the SPI2 peripheral */
    SPI_Enable(SPI2, DISABLE);
}

void SPI2_PrintArd(void){

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

void SPI2_ReadIDArd(void){

    uint8_t dummy_write = 0xFF;
    uint8_t dummy_read = 0;
    uint8_t ack_byte = 0;
    uint8_t cmd_code = 0;
    uint8_t id[11] = {0};

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
        for(int8_t i = 0; i < 10; i++){
            /* Send dummy bits to fetch the response from the slave */
            SPI_SendData(SPI2, &dummy_write, 1);
            SPI_ReceiveData(SPI2, &id[i], 1);
        }
        id[10] = '\0';
        printf("Data read: %s\n", id);
    }

    /* Confirm SPI is not busy */
    while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

    /* Disable the SPI2 peripheral */
    SPI_Enable(SPI2, DISABLE);
}
