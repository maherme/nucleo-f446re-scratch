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
*       void    SPI2_Config(void)
*       void    SPI2_SendHello(void)
*       void    SPI_IRQActions(void)
*       void    SPI_SendCmds(void)
*       void    I2C1_Config(void)
*       void    I2C1_SendHello(void)
*       void    I2C1_SendCmd(void)
*
**/

#ifndef TEST_H
#define TEST_H

#include "spi_driver.h"
#include "i2c_driver.h"

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
 * @fn SPI2_Config
 *
 * @brief function to initialize and configure SPI2 peripheral.
 *
 * @param[in] void
 *
 * @return void
 */
void SPI2_Config(void);

/**
 * @fn SPI2_SendHello
 *
 * @brief function to test SPI2 peripheral transmission.
 *
 * @param[in] void
 *
 * @return void
 */
void SPI2_SendHello(void);

/**
 * @fn SPI_IRQActions
 *
 * @brief function to perform actions needed when interruption from slave raises.
 *
 * @param[in] void
 *
 * @return void
 */
void SPI_IRQActions(void);

/**
 * @fn SPI_SendCmds
 *
 * @brief function to send some commands for testing SPI peripheral.
 *
 * @param[in] void
 *
 * @return void
 */
void SPI_SendCmds(void);

/**
 * @fn I2C1_Config
 *
 * @brief function to initialize and configure I2C1 peripheral.
 *
 * @param[in] void
 *
 * @return void
 */
void I2C1_Config(void);

/**
 * @fn I2C1_SendHello
 *
 * @brief function to test I2C1 peripheral transmission.
 *
 * @param[in] void
 *
 * @return void
 */
void I2C1_SendHello(void);

/**
 * @fn I2C1_SendCmd
 *
 * @brief function to test I2C1 peripheral transmission and reception.
 *
 * @param[in] void
 *
 * @return void
 */

void I2C1_SendCmd(void);

#endif /* TEST_H */
