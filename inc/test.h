/*****************************************************************************************************
* FILENAME :        test.h
*
* DESCRIPTION :
*       Header file containing prototypes of functions for testing drivers.
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
**/

#ifndef TEST_H
#define TEST_H

/**
 * Command codes for SPI test
 */
#define CMD_LED_CTRL        0x50
#define CMD_SENSOR_READ     0x51
#define CMD_LED_READ        0x52
#define CMD_PRINT           0x53
#define CMD_ID_READ         0x54

/**
 * Arduino analog pins
 */
#define ARD_AN_PIN0         0
#define ARD_AN_PIN1         1
#define ARD_AN_PIN2         2
#define ARD_AN_PIN3         3
#define ARD_AN_PIN4         4

/**
 * Arduino GPIO
 */
#define ARD_GPIO_PIN9       9

/**
 * Pin status
 */
#define PIN_ON              1
#define PIN_OFF             0

/**
 * @fn delay
 *
 * @brief function to insert some delay in the execution
 *
 * @param[in] void
 *
 * @return void
 */
void delay(void);

/**
 * @fn LED_GPIOInit
 *
 * @brief function to initialize GPIO port for the LED.
 *
 * @param[in] void
 *
 * @return void
 */
void LED_GPIOInit(void);

/**
 * @fn Button_GPIOInit
 *
 * @brief function to initialize GPIO port for the button.
 *
 * @param[in] void
 *
 * @return void
 */
void Button_GPIOInit(void);

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
void SPI2_GPIOInit(void);

/**
 * @fn SPI2_Init
 *
 * @brief function to initialize SPI2 peripheral.
 *
 * @param[in] void
 *
 * @return void
 */
void SPI2_Init(void);

/**
 * @fn SPI2_SendHello
 *
 * @brief function to send hello through SPI2 peripheral.
 *
 * @param[in] void
 *
 * @return void
 */
void SPI2_SendHello(void);

/**
 * @fn SPI2_SetPinArd
 *
 * @brief function to send command for setting PIN to arduino through SPI2 peripheral.
 *
 * @param[in] void
 *
 * @return void
 */
void SPI2_SetPinArd(void);

/**
 * @fn SPI2_ReadANArd
 *
 * @brief function to send command for reading analog input of arduino through SPI2 peripheral.
 *
 * @param[in] void
 *
 * @return void
 */
void SPI2_ReadANArd(void);

/**
 * @fn SPI2_ReadPinArd
 *
 * @brief function to send command for reading pin status of arduino through SPI2 peripheral.
 *
 * @param[in] void
 *
 * @return void
 */
void SPI2_ReadPinArd(void);

/**
 * @fn SPI2_PrintArd
 *
 * @brief function to send command for printing a message of arduino through SPI2 peripheral.
 *
 * @param[in] void
 *
 * @return void
 */
void SPI2_PrintArd(void);

/**
 * @fn SPI2_PrintArd
 *
 * @brief function to send command for reading the ID of arduino through SPI2 peripheral.
 *
 * @param[in] void
 *
 * @return void
 */
void SPI2_ReadIDArd(void);

#endif /* TEST_H */
