/********************************************************************************************************//**
* @file hil_spi.h
*
* @brief Header file containing the prototypes of the APIs for testing the SPI peripheral in the hardware.
*
* Public Functions:
*       - void    SPI2_Config(void)
*       - void    SPI2_SendHello(void)
*       - void    SPI_IRQActions(void)
*       - void    SPI_SendCmds(void)
**/

#ifndef HIL_SPI_H
#define HIL_SPI_H

/**
 * @name Command codes for SPI test
 * @{
 */
#define CMD_LED_CTRL        0x50    /**< @brief Command for controlling LED */
#define CMD_SENSOR_READ     0x51    /**< @brief Command for reading from sensor of Arduino */
#define CMD_LED_READ        0x52    /**< @brief Command for reading the LED status */
#define CMD_PRINT           0x53    /**< @brief Command for printing message sent by Arduino */
#define CMD_ID_READ         0x54    /**< @brief Command for reading the Arduino ID */
/** @} */

/**
 * @name Arduino analog pins
 * @{
 */
#define ARD_AN_PIN0         0   /**< @brief Analog Pin 0 */
#define ARD_AN_PIN1         1   /**< @brief Analog Pin 1 */
#define ARD_AN_PIN2         2   /**< @brief Analog Pin 2 */
#define ARD_AN_PIN3         3   /**< @brief Analog Pin 3 */
#define ARD_AN_PIN4         4   /**< @brief Analog Pin 4 */
/** @} */

/** @brief Arduino GPIO Pin 9 */
#define ARD_GPIO_PIN9       9

/**
 * @name Pin status
 * @{
 */
#define PIN_ON              1   /**< @brief Pin status ON */
#define PIN_OFF             0   /**< @brief Pin status OFF */
/** @} */

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to initialize and configure SPI2 peripheral.
 * @return void.
 */
void SPI2_Config(void);

/**
 * @brief Function to test SPI2 peripheral transmission.
 * @return void.
 */
void SPI2_SendHello(void);

/**
 * @brief Function to perform actions needed when interruption from slave raises.
 * @return void.
 */
void SPI_IRQActions(void);

/**
 * @brief Function to send some commands for testing SPI peripheral.
 * @return void.
 */
void SPI_SendCmds(void);

#endif /* HIL_SPI_H */
