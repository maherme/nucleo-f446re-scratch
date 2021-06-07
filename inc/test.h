/*****************************************************************************************************
* FILENAME :        test.h
*
* DESCRIPTION :
*       Header file containing prototypes of functions for testing drivers.
*
* PUBLIC FUNCTIONS :
*       void    LED_GPIOInit(void)
*       void    Button_GPIOInit(void)
*       void    SPI2_GPIOInit(void)
*       void    SPI2_Init(void)
*
**/

#ifndef TEST_H
#define TEST_H

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

#endif /* TEST_H */
